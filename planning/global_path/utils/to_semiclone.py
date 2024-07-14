
import pandas as pd
import sys


target = sys.argv[1]
# CSV 파일 읽기 (기본적으로 쉼표(,)를 구분자로 가정)
df = pd.read_csv(f'../paths/{target}.csv')

# CSV 파일 세미콜론(;)을 구분자로 사용하여 저장
df.to_csv(f'/home/kana/catkin_ws/src/mobinha-racing/planning/inputs/traj_ltpl_cl/traj_ltpl_cl_{target}.csv', sep=';', index=False)