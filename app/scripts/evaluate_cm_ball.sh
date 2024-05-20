#!/bin/bash

cpp_program_surface="../../build/app/CenterLineAgreeMesh"

process_files() {
  local inputfolder_xyzr=("$1")
  local inputfolder_mesh=("$2")
  local files=$(find "$inputfolder_xyzr" -type f)
  local total_acc=0
  local total_r_diff=0
  local file_count=0
  for file in $files; do
    # 获取文件名和后缀
    local filename=$(basename "$file")
    local extension="${filename##*.}"

    # 去除后缀的文件名
    local filename_without_extension="${filename%.*}"

    # 指定输出文件路径
    local inputfile_mesh="${inputfolder_mesh}/${filename_without_extension}.ply"
    # 使用编译好的C++程序处理文件，并指定输出文件路径
    local output=$($cpp_program_surface "$file" "$inputfile_mesh")
    local values=($(echo "$output" | tail -n 2))
    # echo $output
    # 检查是否提取到两个值
    if [ ${#values[@]} -eq 2 ]; then
      # 在循环中累加值
      total_acc=$(echo "$total_acc + ${values[0]}" | bc)
      total_r_diff=$(echo "$total_r_diff + ${values[1]}" | bc)
    fi
    file_count=$((file_count + 1))
  done
  local mean_acc=$(echo "scale=4; $total_acc / $file_count" | bc)
  local mean_r_diff=$(echo "scale=4; $total_r_diff / $file_count" | bc)

  echo "总共处理了 $file_count 个文件"
  echo "mean accuracy: $mean_acc"
  echo "mean radius difference: $mean_r_diff"
}

echo "OURS (fast):"
inputfolder_xyzr="/media/wlsdzyzl/DATA/datasets/ATM2022/TrainBatch2_New/output/xyzr"
inputfolder_mesh="/media/wlsdzyzl/DATA/datasets/ATM2022/TrainBatch2_New/output/mesh_from_xyzr_ours_fast"
process_files "$inputfolder_xyzr" "$inputfolder_mesh"

# inputfolder_xyzr="/media/wlsdzyzl/DATA/datasets/CAT08/xyzr"
# inputfolder_mesh="/media/wlsdzyzl/DATA/datasets/CAT08/mesh_from_xyzr_ours_fast"
# process_files "$inputfolder_xyzr" "$inputfolder_mesh"

inputfolder_xyzr="/media/wlsdzyzl/DATA/datasets/DeepVesselNet/output/xyzr"
inputfolder_mesh="/media/wlsdzyzl/DATA/datasets/DeepVesselNet/output/mesh_from_xyzr_ours_fast"
process_files "$inputfolder_xyzr" "$inputfolder_mesh"


inputfolder_xyzr="/media/wlsdzyzl/DATA/datasets/imageCAS/dataset_nii_1000/output/xyzr"
inputfolder_mesh="/media/wlsdzyzl/DATA/datasets/imageCAS/dataset_nii_1000/output/mesh_from_xyzr_ours_fast"
process_files "$inputfolder_xyzr" "$inputfolder_mesh"


inputfolder_xyzr="/media/wlsdzyzl/DATA/datasets/PARSE2022/train/output/xyzr"
inputfolder_mesh="/media/wlsdzyzl/DATA/datasets/PARSE2022/train/output/mesh_from_xyzr_ours_fast"
process_files "$inputfolder_xyzr" "$inputfolder_mesh"

# -------------------------------------------------------------------------------
echo "OURS:"
inputfolder_xyzr="/media/wlsdzyzl/DATA/datasets/ATM2022/TrainBatch2_New/output/xyzr"
inputfolder_mesh="/media/wlsdzyzl/DATA/datasets/ATM2022/TrainBatch2_New/output/mesh_from_xyzr_ours_origin"
process_files "$inputfolder_xyzr" "$inputfolder_mesh"

# inputfolder_xyzr="/media/wlsdzyzl/DATA/datasets/CAT08/xyzr"
# inputfolder_mesh="/media/wlsdzyzl/DATA/datasets/CAT08/mesh_from_xyzr_ours_origin"
# process_files "$inputfolder_xyzr" "$inputfolder_mesh"


inputfolder_xyzr="/media/wlsdzyzl/DATA/datasets/DeepVesselNet/output/xyzr"
inputfolder_mesh="/media/wlsdzyzl/DATA/datasets/DeepVesselNet/output/mesh_from_xyzr_ours_origin"
process_files "$inputfolder_xyzr" "$inputfolder_mesh"


inputfolder_xyzr="/media/wlsdzyzl/DATA/datasets/imageCAS/dataset_nii_1000/output/xyzr"
inputfolder_mesh="/media/wlsdzyzl/DATA/datasets/imageCAS/dataset_nii_1000/output/mesh_from_xyzr_ours_origin"
process_files "$inputfolder_xyzr" "$inputfolder_mesh"

inputfolder_xyzr="/media/wlsdzyzl/DATA/datasets/PARSE2022/train/output/xyzr"
inputfolder_mesh="/media/wlsdzyzl/DATA/datasets/PARSE2022/train/output/mesh_from_xyzr_ours_origin"
process_files "$inputfolder_xyzr" "$inputfolder_mesh"

# -------------------------------------------------------------------------------
echo "POISSON:"
inputfolder_xyzr="/media/wlsdzyzl/DATA/datasets/ATM2022/TrainBatch2_New/output/xyzr"
inputfolder_mesh="/media/wlsdzyzl/DATA/datasets/ATM2022/TrainBatch2_New/output/mesh_from_xyzr_poisson"
process_files "$inputfolder_xyzr" "$inputfolder_mesh"

# inputfolder_xyzr="/media/wlsdzyzl/DATA/datasets/CAT08/xyzr"
# inputfolder_mesh="/media/wlsdzyzl/DATA/datasets/CAT08/mesh_from_xyzr_poisson"
# process_files "$inputfolder_xyzr" "$inputfolder_mesh"

inputfolder_xyzr="/media/wlsdzyzl/DATA/datasets/DeepVesselNet/output/xyzr"
inputfolder_mesh="/media/wlsdzyzl/DATA/datasets/DeepVesselNet/output/mesh_from_xyzr_poisson"
process_files "$inputfolder_xyzr" "$inputfolder_mesh"

inputfolder_xyzr="/media/wlsdzyzl/DATA/datasets/imageCAS/dataset_nii_1000/output/xyzr"
inputfolder_mesh="/media/wlsdzyzl/DATA/datasets/imageCAS/dataset_nii_1000/output/mesh_from_xyzr_poisson"
process_files "$inputfolder_xyzr" "$inputfolder_mesh"

inputfolder_xyzr="/media/wlsdzyzl/DATA/datasets/PARSE2022/train/output/xyzr"
inputfolder_mesh="/media/wlsdzyzl/DATA/datasets/PARSE2022/train/output/mesh_from_xyzr_poisson"
process_files "$inputfolder_xyzr" "$inputfolder_mesh"

# ---------------------------------------------------------------------------------

echo "BALL PIVOTING:"
inputfolder_xyzr="/media/wlsdzyzl/DATA/datasets/ATM2022/TrainBatch2_New/output/xyzr"
inputfolder_mesh="/media/wlsdzyzl/DATA/datasets/ATM2022/TrainBatch2_New/output/mesh_from_xyzr_ballpivoting"
process_files "$inputfolder_xyzr" "$inputfolder_mesh"

# inputfolder_xyzr="/media/wlsdzyzl/DATA/datasets/CAT08/xyzr"
# inputfolder_mesh="/media/wlsdzyzl/DATA/datasets/CAT08/mesh_from_xyzr_ballpivoting"
# process_files "$inputfolder_xyzr" "$inputfolder_mesh"

inputfolder_xyzr="/media/wlsdzyzl/DATA/datasets/DeepVesselNet/output/xyzr"
inputfolder_mesh="/media/wlsdzyzl/DATA/datasets/DeepVesselNet/output/mesh_from_xyzr_ballpivoting"
process_files "$inputfolder_xyzr" "$inputfolder_mesh"

inputfolder_xyzr="/media/wlsdzyzl/DATA/datasets/imageCAS/dataset_nii_1000/output/xyzr"
inputfolder_mesh="/media/wlsdzyzl/DATA/datasets/imageCAS/dataset_nii_1000/output/mesh_from_xyzr_ballpivoting"
process_files "$inputfolder_xyzr" "$inputfolder_mesh"

inputfolder_xyzr="/media/wlsdzyzl/DATA/datasets/PARSE2022/train/output/xyzr"
inputfolder_mesh="/media/wlsdzyzl/DATA/datasets/PARSE2022/train/output/mesh_from_xyzr_ballpivoting"
process_files "$inputfolder_xyzr" "$inputfolder_mesh"
