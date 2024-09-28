import json
import os
import argparse
from libs.ngii2lanelet import NGII2LANELET


def main(args):
    lanelet = NGII2LANELET(
        folder_path=args.ngii_path,
        precision=args.precision,
        base_lla=args.base_lla,
        is_utm=args.is_utm
        )

    name = args.ngii_path.split('/')[-1]

    with open('./maps/%s.json'%(name), 'w', encoding='utf-8') as f:
        json.dump(lanelet.map_data, f, indent="\t")

    with open('./maps/%s_ID.json'%(name), 'w', encoding='utf-8') as f:
        json.dump(lanelet.link_id_data, f, indent="\t")

    pkl_file_path = './pkls/%s.pkl' % (name)
    if os.path.exists(pkl_file_path):
        os.remove(pkl_file_path)

if __name__ == "__main__":
    parser = argparse.ArgumentParser()

    name = 'KIAPI_Racing_Fast'

    parser.add_argument('--ngii_path', type=str, default='./%s'%(name))
    parser.add_argument('--precision', type=float, default=1)
    parser.add_argument('--base_lla', type=tuple, default=(35.65492524,128.39351431,7),help='(lat, lon, alt)')

    #Solchan 37.36549921,126.64108444
    #KIAPI: 35.65492524,128.39351431
    #KCITY: 37.2292221592864,126.76912499027308,29.18400001525879

    parser.add_argument('--is_utm', type=bool, default=True)
    args = parser.parse_args()

    main(args)