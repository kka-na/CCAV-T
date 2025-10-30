from pyproj import Proj, Transformer
from map_config import *

MAP = 'Midan'

blla = get_base_lla(MAP)
proj_wgs84 = Proj(proj='latlong', datum='WGS84') 
proj_enu = Proj(proj='aeqd', datum='WGS84', lat_0=blla[0], lon_0=blla[1], h_0=blla[2])
transformer = Transformer.from_proj(proj_wgs84, proj_enu)

lat1, lng1 = 37.52556408,126.52347784
lat2, lng2 = 37.52555657,126.52346018
lat3, lng3 = 37.52554607,126.52343495

e1,n1,_ = transformer.transform(lng1, lat1, 7)
e2,n2,_ = transformer.transform(lng2, lat2, 7)
e3,n3,_ = transformer.transform(lng3, lat3, 7)

print(f"point1:\n- {e1}\n- {n1}\n\npoint2:\n- {e2}\n- {n2}\n\npoint3:\n- {e3}\n- {n3}")

e = 1354.9522497833113
n = -37.12817746474054

transformer2 = Transformer.from_proj(proj_enu, proj_wgs84)
lon,lat,_ = transformer2.transform(e, n, 7)
print(lat, lon)