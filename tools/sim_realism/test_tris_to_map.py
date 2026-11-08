import numpy as np

import tris_to_map as tm


def box_walls(x0, y0, x1, y1, z0=0., z1=3.):
    """Four wall quads of a rectangle room as triangles."""
    q = lambda a, b: [[[*a, z0], [*b, z0], [*b, z1]], [[*a, z0], [*b, z1], [*a, z1]]]
    return np.array(q((x0, y0), (x1, y0)) + q((x1, y0), (x1, y1)) + q((x1, y1), (x0, y1)) + q((x0, y1), (x0, y0)), float)


def test_a_room_gives_free_inside_occupied_on_the_walls_unknown_outside():
    img, origin, res = tm.rasterize(box_walls(0, 0, 6, 4), seeds=[(3., 2.)], res=.05)
    at = lambda x, y: img[int(round((y - origin[1]) / res)), int(round((x - origin[0]) / res))]
    assert at(3., 2.) == tm.FREE
    assert at(0., 2.) == tm.OCC and at(6., 2.) == tm.OCC and at(3., 4.) == tm.OCC
    assert at(-1., 2.) == tm.UNKNOWN and at(6.5, 2.) == tm.UNKNOWN


def test_only_the_band_is_obstacle():
    low = box_walls(2, 1, 3, 2, z0=0., z1=.05)         # below the band: a floor-level edge
    high = box_walls(4, 1, 5, 2, z0=2.5, z1=3.)        # above the band: a ceiling beam
    img, origin, res = tm.rasterize(np.concatenate([box_walls(0, 0, 6, 4), low, high]), seeds=[(3., 3.)], res=.05, band=(.1, 1.8))
    at = lambda x, y: img[int(round((y - origin[1]) / res)), int(round((x - origin[0]) / res))]
    assert at(2.5, 1.) == tm.FREE and at(4.5, 1.) == tm.FREE


def test_the_yaml_origin_is_the_lower_left_corner_and_the_image_is_flipped():
    img, origin, res = tm.rasterize(box_walls(0, 0, 6, 4), seeds=[(3., 2.)], res=.05)
    pgm = tm.to_pgm(img)
    row_of_y0 = len(pgm) - 1 - int(round((0. - origin[1]) / res))        # the PGM's first row is the highest y
    assert pgm[row_of_y0, int(round((3. - origin[0]) / res))] == 0       # the wall at y = 0
