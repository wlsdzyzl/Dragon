#!/bin/bash

cpp_program_surface="../../build/app/CenterLineAgreeMesh"
inputfolder_xyzr="/media/wlsdzyzl/DATA1/datasets/ATM2022/TrainBatch2_New/output/xyzr"

inputfolder_mesh="/media/wlsdzyzl/DATA1/datasets/ATM2022/TrainBatch2_New/output/mesh_from_xyzr_ours_ball"
files=$(find "$inputfolder_xyzr" -type f)
total_acc=0
file_count=0
for file in $files; do
  # 获取文件名和后缀
  filename=$(basename "$file")
  extension="${filename##*.}"

  # 去除后缀的文件名
  filename_without_extension="${filename%.*}"

  # 指定输出文件路径
  inputfile_mesh="${inputfolder_mesh}/${filename_without_extension}.ply"
  # 使用编译好的C++程序处理文件，并指定输出文件路径
  output=$($cpp_program_surface "$file" "$inputfile_mesh")
  value=$(echo "$output" | tail -n 1)
  total_acc=$(echo "$total_acc + $value" | bc)
  file_count=$((file_count + 1))
done
mean_acc=$(echo "scale=2; $total_acc / $file_count" | bc)
echo "总共处理了 $file_count 个文件"
echo "mean accuracy: $mean_acc"

inputfolder_mesh="/media/wlsdzyzl/DATA1/datasets/ATM2022/TrainBatch2_New/output/mesh_from_xyzr_ballpivoting"
files=$(find "$inputfolder_xyzr" -type f)
total_acc=0
file_count=0
for file in $files; do
  # 获取文件名和后缀
  filename=$(basename "$file")
  extension="${filename##*.}"

  # 去除后缀的文件名
  filename_without_extension="${filename%.*}"

  # 指定输出文件路径
  inputfile_mesh="${inputfolder_mesh}/${filename_without_extension}.ply"
  # 使用编译好的C++程序处理文件，并指定输出文件路径
  output=$($cpp_program_surface "$file" "$inputfile_mesh")
  value=$(echo "$output" | tail -n 1)
  total_acc=$(echo "$total_acc + $value" | bc)
  file_count=$((file_count + 1))
done
mean_acc=$(echo "scale=2; $total_acc / $file_count" | bc)
echo "总共处理了 $file_count 个文件"
echo "mean accuracy: $mean_acc"

inputfolder_mesh="/media/wlsdzyzl/DATA1/datasets/ATM2022/TrainBatch2_New/output/mesh_from_xyzr_poisson"
files=$(find "$inputfolder_xyzr" -type f)
total_acc=0
file_count=0
for file in $files; do
  # 获取文件名和后缀
  filename=$(basename "$file")
  extension="${filename##*.}"

  # 去除后缀的文件名
  filename_without_extension="${filename%.*}"

  # 指定输出文件路径
  inputfile_mesh="${inputfolder_mesh}/${filename_without_extension}.ply"
  # 使用编译好的C++程序处理文件，并指定输出文件路径
  output=$($cpp_program_surface "$file" "$inputfile_mesh")
  value=$(echo "$output" | tail -n 1)
  total_acc=$(echo "$total_acc + $value" | bc)
  file_count=$((file_count + 1))
done
mean_acc=$(echo "scale=2; $total_acc / $file_count" | bc)
echo "总共处理了 $file_count 个文件"
echo "mean accuracy: $mean_acc"