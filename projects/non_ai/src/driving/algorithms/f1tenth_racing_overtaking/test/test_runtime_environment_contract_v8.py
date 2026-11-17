"""Runtime map binding is mandatory for BOTH old and successor R-E nodes."""
import csv
from pathlib import Path
from hashlib import sha256
import yaml
import pytest

from f1tenth_racing_overtaking.runtime_environment import load_verified_runtime


def mk(tmp_path):
    root=tmp_path/'sim'
    (root/'maps').mkdir(parents=True)
    (root/'maps/track.pgm').write_bytes(b'P5\n1 1\n255\n\x00')
    with (root/'maps/line.csv').open('w',newline='') as f:
        w=csv.writer(f);w.writerow(['x_m','y_m','w_tr_left_m','w_tr_right_m'])
        w.writerows([[0,0,1,1],[1,0,1,1],[1,1,1,1]])
    img=root/'maps/track.pgm'; line=root/'maps/line.csv'
    env={'environment_identity':{'active_track':'spielberg'},'map':{
        'image_file':'maps/track.pgm','image_sha256':sha256(img.read_bytes()).hexdigest(),
        'centerline_file':'maps/line.csv','centerline_sha256':sha256(line.read_bytes()).hexdigest()}}
    path=root/'config.yaml';path.write_text(yaml.safe_dump(env))
    return root,path,line,img,env


def verify(root,path,line,img,env,**kwargs):
    return load_verified_runtime(path,kwargs.pop('centerline_file',''),root,**kwargs)


def test_known_good(tmp_path):
    root,path,line,img,env=mk(tmp_path)
    actual,rows=verify(root,path,line,img,env,expected_track_id='spielberg',
                     expected_map_sha256=env['map']['image_sha256'])
    assert actual==env and len(rows)==3


def test_explicit_centerline_override_same_sha(tmp_path):
    root,path,line,img,env=mk(tmp_path)
    _,rows=verify(root,path,line,img,env,centerline_file=str(line))
    assert len(rows)==3


def test_track_swap_rejected(tmp_path):
    root,path,line,img,env=mk(tmp_path)
    with pytest.raises(ValueError,match='TRACK_MISMATCH'):
        verify(root,path,line,img,env,expected_track_id='iccas2025')


def test_map_identity_swap_rejected(tmp_path):
    root,path,line,img,env=mk(tmp_path)
    with pytest.raises(ValueError,match='MAP_IDENTITY_MISMATCH'):
        verify(root,path,line,img,env,expected_map_sha256='0'*64)


def test_map_on_disk_tamper_rejected(tmp_path):
    root,path,line,img,env=mk(tmp_path);img.write_bytes(b'injected')
    with pytest.raises(ValueError,match='MAP_FILE_SHA_MISMATCH'):
        verify(root,path,line,img,env)


def test_centerline_on_disk_tamper_rejected(tmp_path):
    root,path,line,img,env=mk(tmp_path);line.write_bytes(b'injected')
    with pytest.raises(ValueError,match='CENTERLINE_SHA_MISMATCH'):
        verify(root,path,line,img,env)


def test_centerline_override_wrong_sha_rejected(tmp_path):
    root,path,line,img,env=mk(tmp_path);other=root/'other.csv';other.write_text('invalid')
    with pytest.raises(ValueError,match='CENTERLINE_SHA_MISMATCH'):
        verify(root,path,line,img,env,centerline_file=str(other))


@pytest.mark.parametrize('rows',[[],[[0,0,1,1]],[[0,0,1,1],[1,0,1,1]]])
def test_short_centerline_rejected(tmp_path,rows):
    root,path,line,img,env=mk(tmp_path)
    with line.open('w',newline='') as f:
        w=csv.writer(f);w.writerow(['x_m','y_m','w_tr_left_m','w_tr_right_m']);w.writerows(rows)
    env['map']['centerline_sha256']=sha256(line.read_bytes()).hexdigest()
    path.write_text(yaml.safe_dump(env))
    with pytest.raises(ValueError,match='CENTERLINE_TOO_SHORT'):
        verify(root,path,line,img,env)


@pytest.mark.parametrize('bad_value',['nan','inf','-inf','0','-1'])
def test_bad_track_width_rejected(tmp_path,bad_value):
    root,path,line,img,env=mk(tmp_path)
    text=line.read_text().replace('0,0,1,1','0,0,'+bad_value+',1')
    line.write_text(text)
    env['map']['centerline_sha256']=sha256(line.read_bytes()).hexdigest()
    path.write_text(yaml.safe_dump(env))
    with pytest.raises(ValueError,match='INVALID_GEOMETRY'):
        verify(root,path,line,img,env)


def test_existing_r_e_bind_both_maps():
    root=Path(__file__).resolve().parents[4]
    sim=root/'environment/f1tenth_racing_sim'
    for name in ('competition','spielberg'):
        environment,rows=load_verified_runtime(sim/'config'/f'{name}_environment.yaml','',sim)
        assert environment['environment_identity']['active_track'] in ('iccas2025','spielberg')
        assert len(rows)>=66
