from pyproj import Proj, Transformer

def geodetic2enu(lat, lon, h, lat0, lon0, h0):
    # WGS84 좌표계를 사용합니다.
    proj_wgs84 = Proj(proj='latlong', datum='WGS84') 

    # ENU 좌표계를 정의합니다. (lat0, lon0, h0)는 기준점의 좌표입니다.
    proj_enu = Proj(proj='aeqd', datum='WGS84', lat_0=lat0, lon_0=lon0, h_0=h0)

    # Transformer 객체를 사용하여 변환기를 생성합니다.
    transformer = Transformer.from_proj(proj_wgs84, proj_enu)

    # WGS84 좌표를 ENU 좌표로 변환합니다.
    e, n, u = transformer.transform(lon, lat, h)

    return e, n, u