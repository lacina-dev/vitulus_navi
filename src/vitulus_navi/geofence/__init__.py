"""Robot-native geofence proposal writer (propose-only, no signing).

This subpackage is the robot's own, public home for turning a clicked map
perimeter into an UNSIGNED geofence proposal.  It exists so the public web
node (`vitulus_ui/nodes/webnode`, :7779) can serve `POST /api/geofence/propose`
WITHOUT importing anything from the private agent (`vitulus_claude`).

Layout:
  - ``fence.py``      — verbatim copy of the agent's ``geofence.py`` (the
                        safety geometry / `Fence` validator; pure stdlib).
  - ``sitebundle.py`` — verbatim copy of the agent's ``sitebundle.py``.
  - ``propose.py``    — propose-only port of the agent's ``geofence_propose.py``
                        (``build_proposal`` / ``write_proposal`` etc.); the
                        signing functions are intentionally absent.

Because ``fence.py`` is a verbatim copy of the agent's validator, the ring
digest and the on-disk proposal format are identical to the agent path.

Signing is NOT here on purpose: an active ``geofence.geojson`` can only be
produced by a human running the agent's ``tools/geofence_propose sign``
(or ``/geofence podepsat <digest>``).  Nothing in this subpackage — and so no
HTTP request to the web node — can arm a fence.
"""
from . import propose  # noqa: F401
from .propose import (  # noqa: F401
    ProposalError,
    build_proposal,
    write_proposal,
    summarize,
    proposal_path,
    active_path,
)
from .fence import GeofenceError  # noqa: F401
