
import pandas as pd

# CSV 파일 읽기 (기본적으로 쉼표(,)를 구분자로 가정)
df = pd.read_csv('./paths/test.csv')

# CSV 파일 세미콜론(;)을 구분자로 사용하여 저장
df.to_csv('../TUMLocalPath/inputs/traj_ltpl_cl/traj_ltpl_cl_test.csv', sep=';', index=False)