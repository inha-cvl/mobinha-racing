import json
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

    with open('%s.json'%(name), 'w', encoding='utf-8') as f:
        json.dump(lanelet.map_data, f, indent="\t")

    with open('%s_ID.json'%(name), 'w', encoding='utf-8') as f:
        json.dump(lanelet.link_id_data, f, indent="\t")


if __name__ == "__main__":
    parser = argparse.ArgumentParser()

    name = 'KIAPI_Racing'

    parser.add_argument('--ngii_path', type=str, default='./%s'%(name))
    parser.add_argument('--precision', type=float, default=1)
    parser.add_argument('--base_lla', type=tuple, default=(35.64750540757964,128.40264207604886,7),help='(lat, lon, alt)')
    
        #35.64750540757964,128.40264207604886,7),help='(lat, lon, alt)')
       # 37.39991792889962, 127.11264200835348, 7), 
    #37.42390324724057, 126.60753475932731 : HARBOR

    parser.add_argument('--is_utm', type=bool, default=True)
    args = parser.parse_args()

    main(args)