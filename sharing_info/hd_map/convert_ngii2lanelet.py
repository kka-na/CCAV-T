import json
import os
from libs.ngii2lanelet import NGII2LANELET

if __name__ == "__main__":

    name = 'Midan'
    path = '/home/kana/Documents/Dataset/MAP/Midan'
    precision = 1
    base_lla = (37.5272470,126.5068108, 7)
    is_utm = False
   
    lanelet = NGII2LANELET(path, precision, base_lla, is_utm)

    with open('./maps/%s.json'%(name), 'w', encoding='utf-8') as f:
        json.dump(lanelet.map_data, f, indent="\t")

    pkl_file_path = './pkls/%s.pkl' % (name)
    if os.path.exists(pkl_file_path):
        os.remove(pkl_file_path)