from pyproj import Proj, transform

# EPSG 5181 (Korea Central Belt 1985) 정의
proj_5181 = Proj(init='epsg:5181')

# WGS84
proj_utm = Proj(init='epsg:4326')

# 변환할 좌표 (x, y)를 EPSG 5181 기준으로 설정합니다.
x_5181, y_5181 = 209973.70,433402.36

#14093886.3,4498359.3 : HARBOR
#327031.89, 239839.66 : KIAPI

lon,lat = transform(proj_5181, proj_utm, x_5181, y_5181)

print(f"UTM 좌표: {lat}, {lon}")
