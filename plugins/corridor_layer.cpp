// Corridor preference layer of the GLOBAL costmap (T36, switch map_corridor_prefer).
//
// GlobalPlanner pays  min(neutral_cost + cost_factor * c, lethal_cost - 1)  for a
// cell of cost c, and a cell can never cost it less than neutral_cost. To make
// the corridors the owner drew CHEAPER and leave everything else EXACTLY as it
// is, config/global_costmap_corridor.yaml starts the planner with a lower
// neutral_cost and cost_factor (38 and 1.5 instead of 50 and 3.0) and this
// layer, the last one of the global costmap (after inflation), gives every
// passable cell the cost c' with which the planner pays for it
//   outside the corridors   exactly what it pays without the switch
//                           (38 + 1.5 * (2c + 8) = 50 + 3c, the same ceiling 252),
//   inside a corridor       k' * v less (v = the cell of /navi_manager/corridor_cost,
//                           7: 39.5 + 3c), never less than neutral' + 3c.
// So plans that gain nothing from a corridor are priced cell by cell as today:
// the same distance from obstacles, the same choice between a narrow gap and a
// way round (review A680 N1, A691 N4), the same docking. A way along a corridor
// may be 50 / 39.5 = 1.27 times longer than the direct one and still wins.
//
// The layer only RAISES costs and never to lethal_cost - 1 = 252 or more (the
// planner takes that as an obstacle): a passable cell stays passable, a cell of
// 252 and more (obstacle, inscribed, unknown) is never touched. A corridor
// across a nogo area or an obstacle opens nothing.
//
// /navi_manager/corridor_cost has the geometry of /navi_manager/map: v > 0 in a
// corridor, 0 elsewhere. No grid, an empty grid, a grid that does not fit the
// costmap (the map changed and the new grid is on its way): no corridor, every
// cell priced as without the switch. The layer never resizes the costmap and
// never waits for its topic.
#include <algorithm>
#include <cmath>
#include <string>

#include <boost/thread/mutex.hpp>
#include <costmap_2d/cost_values.h>
#include <costmap_2d/layer.h>
#include <costmap_2d/layered_costmap.h>
#include <nav_msgs/OccupancyGrid.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

namespace vitulus_navi
{

// The planner's price of a cell (global_planner Expander::getCost).
struct Price
{
  double neutral, factor;
  int lethal;
  bool passable(int c) const { return c < lethal - 1; }
  double of(int c) const
  {
    if (!passable(c))
      return lethal;
    const double p = c * factor + neutral;
    return p >= lethal ? lethal - 1.0 : p;
  }
};

// c' of a master cell c: `now` = the planner as it runs, `ref` = the planner
// without the switch, v = the corridor cell (0 = not in a corridor).
inline int corridorCell(int c, int v, const Price& now, const Price& ref)
{
  if (c < 0 || c > 255 || !ref.passable(c) || !now.passable(c))
    return c;                              // obstacle, inscribed, unknown: as it is
  double want = ref.of(c);                 // what the planner pays without the switch
  if (v > 0)                               // a corridor: v cells' worth less, the slope of the inflation stays
    want = std::min(want, std::max(want - now.factor * v, now.neutral + ref.factor * c));
  // the cheapest cost, not under c, with which the planner pays that (or the nearest to it)
  int best = c;
  double miss = std::fabs(now.of(c) - want);
  for (int k = c + 1; now.passable(k) && k < 256; k++)
  {
    const double m = std::fabs(now.of(k) - want);
    if (m < miss - 1e-9)
    {
      best = k;
      miss = m;
    }
  }
  return best;
}

// The planner's values without the switch (config/global_planner_params.yaml; who changes them
// there changes them here and in config/global_costmap_corridor.yaml, test/test_map_corridor.py
// holds the three together).
static const double TODAY_NEUTRAL = 50.0;
static const double TODAY_FACTOR = 3.0;
static const int TODAY_LETHAL = 253;

// How far the planner `now` is from paying, outside the corridors, what `ref` pays (0 = exact).
inline double tableError(const Price& now, const Price& ref)
{
  double worst = 0.0;
  for (int c = 0; c < 256; c++)
    worst = std::max(worst, std::fabs(now.of(corridorCell(c, 0, now, ref)) - ref.of(c)));
  return worst;
}

class CorridorLayer : public costmap_2d::Layer
{
public:
  static const int MAX_V = 60;

  CorridorLayer() : changed_(true), off_(false) {}

  virtual void onInitialize()
  {
    ros::NodeHandle nh("~/" + name_);
    std::string topic;
    nh.param("topic", topic, std::string("/navi_manager/corridor_cost"));
    // the planner as it runs now (config/global_costmap_corridor.yaml) ...
    ros::NodeHandle planner("~/GlobalPlanner");
    Price now, ref;
    int neutral;
    planner.param("neutral_cost", neutral, static_cast<int>(TODAY_NEUTRAL));
    now.neutral = neutral;
    planner.param("cost_factor", now.factor, TODAY_FACTOR);
    planner.param("lethal_cost", now.lethal, TODAY_LETHAL);
    // ... and as it runs without the switch: fixed here, not a parameter. A missing or wrong
    // reference would make the table "change nothing" while the planner quietly pays
    // 38 + 1.5 c and plans closer to obstacles (review A714).
    ref.neutral = TODAY_NEUTRAL;
    ref.factor = TODAY_FACTOR;
    ref.lethal = TODAY_LETHAL;
    double worst = tableError(now, ref);
    if (worst > 1e-6)
    {
      // Not exact: this layer must not run with this planner. The planner is created after the
      // costmaps, so it still reads its parameters: give it back the values without the switch
      // and change no cell. Planning is then exactly as without the switch.
      ROS_ERROR("CorridorLayer: with neutral_cost %.0f and cost_factor %.3f the planner cannot pay exactly what it "
                "pays with %.0f and %.3f (cells outside the corridors would differ by up to %.2f). The corridor "
                "preference is OFF: the planner gets %.0f and %.3f back and the layer changes nothing.",
                now.neutral, now.factor, ref.neutral, ref.factor, worst, ref.neutral, ref.factor);
      planner.setParam("neutral_cost", static_cast<int>(std::lround(ref.neutral)));
      planner.setParam("cost_factor", ref.factor);
      planner.setParam("lethal_cost", ref.lethal);
      now = ref;
      now.neutral = std::lround(ref.neutral);
      off_ = true;
    }
    for (int c = 0; c < 256; c++)
      for (int v = 0; v <= MAX_V; v++)
        lut_[v][c] = static_cast<unsigned char>(off_ ? c : corridorCell(c, v, now, ref));
    // always on: the planner's lower neutral_cost needs this layer (no "enabled" parameter on purpose)
    enabled_ = true;
    current_ = true;
    changed_ = true;
    sub_ = nh.subscribe(topic, 1, &CorridorLayer::incoming, this);
    if (!off_)
      ROS_INFO("CorridorLayer: corridors from %s; planner %.0f + %.2f * c, outside the corridors as %.0f + %.2f * c, "
               "a corridor cell of 7 costs %.1f",
               topic.c_str(), now.neutral, now.factor, ref.neutral, ref.factor, now.of(lut_[7][0]));
  }

  virtual void updateBounds(double, double, double, double* min_x, double* min_y,
                            double* max_x, double* max_y)
  {
    boost::mutex::scoped_lock lock(mutex_);
    if (!changed_)
      return;
    changed_ = false;
    // a new grid (or none any more): the whole map once
    const costmap_2d::Costmap2D* m = layered_costmap_->getCostmap();
    *min_x = std::min(*min_x, m->getOriginX());
    *min_y = std::min(*min_y, m->getOriginY());
    *max_x = std::max(*max_x, m->getOriginX() + m->getSizeInMetersX());
    *max_y = std::max(*max_y, m->getOriginY() + m->getSizeInMetersY());
  }

  virtual void updateCosts(costmap_2d::Costmap2D& master, int min_i, int min_j, int max_i, int max_j)
  {
    boost::mutex::scoped_lock lock(mutex_);
    unsigned char* cells = master.getCharMap();
    const unsigned int nx = master.getSizeInCellsX();
    const signed char* data = 0;
    if (usable())
    {
      const nav_msgs::MapMetaData& g = grid_->info;
      if (g.width != master.getSizeInCellsX() || g.height != master.getSizeInCellsY() ||
          std::fabs(g.resolution - master.getResolution()) > 1e-6 ||
          std::fabs(g.origin.position.x - master.getOriginX()) > 1e-6 ||
          std::fabs(g.origin.position.y - master.getOriginY()) > 1e-6)
        ROS_WARN_THROTTLE(60.0, "CorridorLayer: the corridor grid does not fit the costmap, no corridor used");
      else
        data = &grid_->data[0];
    }
    for (int j = min_j; j < max_j; j++)
    {
      unsigned int k = j * nx + min_i;
      for (int i = min_i; i < max_i; i++, k++)
      {
        const int v = data ? std::min<int>(data[k], MAX_V) : 0;
        cells[k] = lut_[v > 0 ? v : 0][cells[k]];
      }
    }
  }

private:
  bool usable() const
  {
    return grid_ && grid_->info.width > 0 && grid_->info.height > 0 &&
           grid_->data.size() == static_cast<size_t>(grid_->info.width) * grid_->info.height;
  }

  void incoming(const nav_msgs::OccupancyGridConstPtr& msg)
  {
    boost::mutex::scoped_lock lock(mutex_);
    grid_ = msg;
    changed_ = true;
  }

  ros::Subscriber sub_;
  nav_msgs::OccupancyGridConstPtr grid_;
  boost::mutex mutex_;
  bool changed_;       // a grid came since the last updateBounds
  bool off_;           // the planner's parameters do not fit: nothing is changed
  unsigned char lut_[MAX_V + 1][256];
};

}  // namespace vitulus_navi

// navi_man asks for this before it lets move_base_flex load the layer
// (map_corridor.LIBRARY_VERSION): a library that is missing, does not load or
// is an older build keeps the switch off instead of taking the planner down.
extern "C" int vitulus_navi_corridor_layer_version()
{
  return 4;
}

// the layer's arithmetic for test/test_map_corridor.py
extern "C" int vitulus_navi_corridor_layer_cell(int c, int v, double neutral, double factor,
                                                double as_neutral, double as_factor, int lethal)
{
  vitulus_navi::Price now = {neutral, factor, lethal}, ref = {as_neutral, as_factor, lethal};
  return vitulus_navi::corridorCell(c, std::min(v, static_cast<int>(vitulus_navi::CorridorLayer::MAX_V)), now, ref);
}

// 0 = with this planner the layer runs; else how far the prices outside the corridors would be off.
extern "C" double vitulus_navi_corridor_layer_table_error(double neutral, double factor, int lethal)
{
  vitulus_navi::Price now = {neutral, factor, lethal};
  vitulus_navi::Price ref = {vitulus_navi::TODAY_NEUTRAL, vitulus_navi::TODAY_FACTOR, vitulus_navi::TODAY_LETHAL};
  return vitulus_navi::tableError(now, ref);
}

extern "C" double vitulus_navi_corridor_layer_price(int c, double neutral, double factor, int lethal)
{
  vitulus_navi::Price p = {neutral, factor, lethal};
  return p.of(c);
}

PLUGINLIB_EXPORT_CLASS(vitulus_navi::CorridorLayer, costmap_2d::Layer)
