
# get all the folders from /home/npopkov/repos/IR2025/data/
import os
from tqdm import tqdm

def get_data_folders(data_path='/home/npopkov/repos/IR2025/data/'):
    folders = []
    for entry in os.listdir(data_path):
        full_path = os.path.join(data_path, entry)
        if os.path.isdir(full_path):
            folders.append(full_path)
    return folders

if __name__ == "__main__":
    data_folders = get_data_folders()

    # get sudo access
    os.system('sudo -v')

    for folder in tqdm(data_folders):
        print(folder)
        #run /bin/python3 /home/npopkov/repos/IR2025/scen/data.py --car_dir "$folder"
        os.system(f'/bin/python3 /home/npopkov/repos/IR2025/scen/data.py --car_dir "{folder}"')
        #after that scp -r $folder cluster.ies:/mnt/work/verano/data/20251128_1241_10v_150w_3sp/
        os.system(f'scp -r {folder} cluster.ies:/mnt/work/verano/data/')
        #after that rm -rf $folder
        os.system(f'rm -rf {folder}')

    print("All folders processed.")

