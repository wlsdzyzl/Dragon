import os
import subprocess
import SimpleITK as sitk
import numpy as np
def save_itk(filename, imageArray, origin = None, spacing = None):
    itkimage = sitk.GetImageFromArray(imageArray)
    if origin is not None:
        itkimage.SetOrigin(origin)
    if spacing is not None:
        itkimage.SetSpacing(spacing)
    sitk.WriteImage(itkimage, filename, useCompression = True)
def npy2nii(npy_file, nii_file):
    data = np.load(npy_file)
    save_itk(nii_file, data)
def get_truncation(xyzr_path):
    numbers = []  # 存储每一行的最后一个数的数组

    with open(xyzr_path, 'r') as file:
        for line in file:
            line = line.strip()  # 去除行末尾的换行符和空格
            if line:  # 忽略空行
                last_number = float(line.split()[-1])  # 提取最后一个数并转换为浮点数
                numbers.append(last_number)

    if numbers:
        max_number = max(numbers)
        return max_number
    else:
        return None
def process_file(file_path, input_path, output_path, xyzr_path):
    # 读取文件大小
    image = sitk.ReadImage(file_path)
    size = image.GetSize()
    # 调用C++程序
    cpp_program_path = "../../build/app/Mesh2Indicator" # 替换为你的C++程序路径
    file_name = os.path.splitext(os.path.basename(file_path))[0]
    file_name = os.path.splitext(os.path.basename(file_name))[0]
    trunc = get_truncation(os.path.join(xyzr_path, file_name+".xyzr"))
    input_file_path = os.path.join(input_path, file_name+".ply")
    output_file_path = os.path.join(output_path, file_name + '.npy')
    command = [cpp_program_path, input_file_path, output_file_path, str(size[2]), str(size[1]), str(size[0]), "1.0", str(trunc), '0']
    print(command)
    subprocess.call(command)
    npy2nii(output_file_path, os.path.join(output_path, file_name + '.nii.gz'))
    return size
# input_path: ply path
# output_path: npy path
def process_directory(directory_path, input_path, output_path, xyzr_path):
    # 遍历文件夹下的所有.nii.gz文件
    min_size = [1e7, 1e7, 1e7]
    max_size = [-1e7, -1e7, -1e7]
    for root, dirs, files in os.walk(directory_path):
        for file in files:
            if file.endswith('.nii.gz'):
                file_path = os.path.join(root, file)
                size = process_file(file_path, input_path, output_path, xyzr_path)
                if size[0] > max_size[0]: max_size[0] = size[0]
                if size[0] < min_size[0]: min_size[0] = size[0]
                if size[1] > max_size[1]: max_size[1] = size[1]
                if size[1] < min_size[1]: min_size[1] = size[1]       
                if size[2] > max_size[2]: max_size[2] = size[2]
                if size[2] < min_size[2]: min_size[2] = size[2]           
    print('-----------------------------------------------------')
    print(min_size)
    print(max_size)

                


# method: ours
# atm
directory_path = '/media/wlsdzyzl/DATA/datasets/ATM2022/TrainBatch2_New/labelsTr/'  # 替换为你的文件夹路径
input_path = "/media/wlsdzyzl/DATA/datasets/ATM2022/TrainBatch2_New/output/mesh_from_xyzr_ours_fast/"
output_path = "/media/wlsdzyzl/DATA/datasets/ATM2022/TrainBatch2_New/output/mask_fine_ours_ball/"
xyzr_path = "/media/wlsdzyzl/DATA/datasets/ATM2022/TrainBatch2_New/output/xyzr"

process_directory(directory_path, input_path, output_path, xyzr_path)

# imagecas
directory_path = '/media/wlsdzyzl/DATA/datasets/imageCAS/dataset_nii_1000/label/'  # 替换为你的文件夹路径
input_path = "/media/wlsdzyzl/DATA/datasets/imageCAS/dataset_nii_1000/output/mesh_from_xyzr_ours_fast"
output_path = "/media/wlsdzyzl/DATA/datasets/imageCAS/dataset_nii_1000/output/mask_fine_ours_ball/"
xyzr_path = "/media/wlsdzyzl/DATA/datasets/imageCAS/dataset_nii_1000/output/xyzr"

process_directory(directory_path, input_path, output_path, xyzr_path)

# parse
directory_path = '/media/wlsdzyzl/DATA/datasets/PARSE2022/train/label/'  # 替换为你的文件夹路径
input_path = "/media/wlsdzyzl/DATA/datasets/PARSE2022/train/output/mesh_from_xyzr_ours_fast"
output_path = "/media/wlsdzyzl/DATA/datasets/PARSE2022/train/output/mask_fine_ours_ball/"
xyzr_path = "/media/wlsdzyzl/DATA/datasets/PARSE2022/train/output/xyzr"

process_directory(directory_path, input_path, output_path, xyzr_path)

# deep
directory_path = '/media/wlsdzyzl/DATA/datasets/DeepVesselNet/seg/'  # 替换为你的文件夹路径
input_path = "/media/wlsdzyzl/DATA/datasets/DeepVesselNet/output/mesh_from_xyzr_ours_fast"
output_path = "/media/wlsdzyzl/DATA/datasets/DeepVesselNet/output/mask_fine_ours_ball/"
xyzr_path = "/media/wlsdzyzl/DATA/datasets/DeepVesselNet/output/xyzr"

process_directory(directory_path, input_path, output_path, xyzr_path)


# # method: ball pivoting
# # atm
# directory_path = '/media/wlsdzyzl/DATA/datasets/ATM2022/TrainBatch2_New/labelsTr/'  # 替换为你的文件夹路径
# input_path = "/media/wlsdzyzl/DATA/datasets/ATM2022/TrainBatch2_New/output/mesh_from_xyzr_ballpivoting/"
# output_path = "/media/wlsdzyzl/DATA/datasets/ATM2022/TrainBatch2_New/output/mask_fine_ballpivoting/"
# xyzr_path = "/media/wlsdzyzl/DATA/datasets/ATM2022/TrainBatch2_New/output/xyzr"

# process_directory(directory_path, input_path, output_path, xyzr_path)

# # imagecas
# directory_path = '/media/wlsdzyzl/DATA/datasets/imageCAS/dataset_nii_1000/label/'  # 替换为你的文件夹路径
# input_path = "/media/wlsdzyzl/DATA/datasets/imageCAS/dataset_nii_1000/output/mesh_from_xyzr_ballpivoting"
# output_path = "/media/wlsdzyzl/DATA/datasets/imageCAS/dataset_nii_1000/output/mask_fine_ballpivoting/"
# xyzr_path = "/media/wlsdzyzl/DATA/datasets/imageCAS/dataset_nii_1000/output/xyzr"

# process_directory(directory_path, input_path, output_path, xyzr_path)

# # parse
# directory_path = '/media/wlsdzyzl/DATA/datasets/PARSE2022/train/label/'  # 替换为你的文件夹路径
# input_path = "/media/wlsdzyzl/DATA/datasets/PARSE2022/train/output/mesh_from_xyzr_ballpivoting"
# output_path = "/media/wlsdzyzl/DATA/datasets/PARSE2022/train/output/mask_fine_ballpivoting/"
# xyzr_path = "/media/wlsdzyzl/DATA/datasets/PARSE2022/train/output/xyzr"

# process_directory(directory_path, input_path, output_path, xyzr_path)

# # deep
# directory_path = '/media/wlsdzyzl/DATA/datasets/DeepVesselNet/seg/'  # 替换为你的文件夹路径
# input_path = "/media/wlsdzyzl/DATA/datasets/DeepVesselNet/output/mesh_from_xyzr_ballpivoting"
# output_path = "/media/wlsdzyzl/DATA/datasets/DeepVesselNet/output/mask_fine_ballpivoting/"
# xyzr_path = "/media/wlsdzyzl/DATA/datasets/DeepVesselNet/output/xyzr"

# process_directory(directory_path, input_path, output_path, xyzr_path)


# # method: poisson
# # atm
# directory_path = '/media/wlsdzyzl/DATA/datasets/ATM2022/TrainBatch2_New/labelsTr/'  # 替换为你的文件夹路径
# input_path = "/media/wlsdzyzl/DATA/datasets/ATM2022/TrainBatch2_New/output/mesh_from_xyzr_poisson/"
# output_path = "/media/wlsdzyzl/DATA/datasets/ATM2022/TrainBatch2_New/output/mask_fine_poisson/"
# xyzr_path = "/media/wlsdzyzl/DATA/datasets/ATM2022/TrainBatch2_New/output/xyzr"

# process_directory(directory_path, input_path, output_path, xyzr_path)

# # imagecas
# directory_path = '/media/wlsdzyzl/DATA/datasets/imageCAS/dataset_nii_1000/label/'  # 替换为你的文件夹路径
# input_path = "/media/wlsdzyzl/DATA/datasets/imageCAS/dataset_nii_1000/output/mesh_from_xyzr_poisson"
# output_path = "/media/wlsdzyzl/DATA/datasets/imageCAS/dataset_nii_1000/output/mask_fine_poisson/"
# xyzr_path = "/media/wlsdzyzl/DATA/datasets/imageCAS/dataset_nii_1000/output/xyzr"

# process_directory(directory_path, input_path, output_path, xyzr_path)

# # parse
# directory_path = '/media/wlsdzyzl/DATA/datasets/PARSE2022/train/label/'  # 替换为你的文件夹路径
# input_path = "/media/wlsdzyzl/DATA/datasets/PARSE2022/train/output/mesh_from_xyzr_poisson"
# output_path = "/media/wlsdzyzl/DATA/datasets/PARSE2022/train/output/mask_fine_poisson/"
# xyzr_path = "/media/wlsdzyzl/DATA/datasets/PARSE2022/train/output/xyzr"

# process_directory(directory_path, input_path, output_path, xyzr_path)

# # deep
# directory_path = '/media/wlsdzyzl/DATA/datasets/DeepVesselNet/seg/'  # 替换为你的文件夹路径
# input_path = "/media/wlsdzyzl/DATA/datasets/DeepVesselNet/output/mesh_from_xyzr_poisson"
# output_path = "/media/wlsdzyzl/DATA/datasets/DeepVesselNet/output/mask_fine_poisson/"
# xyzr_path = "/media/wlsdzyzl/DATA/datasets/DeepVesselNet/output/xyzr"

# process_directory(directory_path, input_path, output_path, xyzr_path)