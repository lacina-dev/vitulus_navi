import re
import subprocess

import numpy as np
import matplotlib.image as mpimg

# Stole this function from
# http://stackoverflow.com/questions/7368739/numpy-and-16-bit-pgm/7369986#7369986
def read_pgm(filename, byteorder='>'):
    """Return image data from a raw PGM file as numpy array.
    Format specification: http://netpbm.sourceforge.net/doc/pgm.html
    """
    with open(filename, 'rb') as f:
        buffer_ = f.read()
    try:
        header, width, height, maxval = re.search(
            b"(^P5\s(?:\s*#.*[\r\n])*"
            b"(\d+)\s(?:\s*#.*[\r\n])*"
            b"(\d+)\s(?:\s*#.*[\r\n])*"
            b"(\d+)\s(?:\s*#.*[\r\n]\s)*)", buffer_).groups()
    except AttributeError:
        raise ValueError("Not a raw PGM file: '%s'" % filename)
    return np.frombuffer(buffer_,
                         dtype='u1' if int(maxval) < 256 else byteorder+'u2',
                         count=int(width)*int(height),
                         offset=len(header)
                         ).reshape((int(height), int(width)))

np_mask = read_pgm("Map_baseMask.pgm")
print(np_mask.shape)
print(len(np_mask))
r = 0
for row in np_mask:
    print(r)
    r += 1
    c = 0
    for col in row:
        print(c)
        c += 1