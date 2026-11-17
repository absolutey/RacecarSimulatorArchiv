from pathlib import Path
import math
from types import SimpleNamespace
from f1tenth_racing_sim.world.map import OccupancyMap


def test_rotated_grid_coordinates(tmp_path):
    # Bottom-left image coordinate (col=0,row_from_bottom=0) free, others occupied.
    pgm=tmp_path/'m.pgm'
    pgm.write_bytes(b'P5\n2 2\n255\n'+bytes([255,0,0,0]))
    config={'resolution_m_per_pixel':1.0,'origin':{'x_m':10,'y_m':20,'yaw_rad':math.pi/2},
            'runtime_pixel_occupied_below':128,'runtime_flip_y':True}
    grid=OccupancyMap(pgm,config)
    assert grid.origin_yaw == math.pi/2
    # Local (0.25,0.25) rotates to global (9.75,20.25): local image bottom left occupied
    assert grid.occupied(9.75,20.25)
    # Local (0.25,1.25) -> global (8.75,20.25): image top left free
    assert not grid.occupied(8.75,20.25)
    assert grid.occupied(0,0)
