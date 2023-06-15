#!/bin/bash

cpp_program_surface="../../build/app/CenterLine2SurfacePoints"
python_program_possion="./poisson_recon.py"
inputfolder="/media/wlsdzyzl/DATA/datasets/ATM2022/TrainBatch2_New/output/xyzr"
outputfolder="/media/wlsdzyzl/DATA/datasets/ATM2022/TrainBatch2_New/output/mesh_from_xyzr_poisson"
files=$(find "$inputfolder" -type f)
total_time=0
file_count=0
for file in $files; do
  # 获取文件名和后缀
  filename=$(basename "$file")
  extension="${filename##*.}"

  # 去除后缀的文件名
  filename_without_extension="${filename%.*}"

  # 指定输出文件路径
  output_file="${outputfolder}/${filename_without_extension}.ply"
  start_time=$(date +%s.%N)
  # 使用编译好的C++程序处理文件，并指定输出文件路径
  $cpp_program_surface "$file" "./surf.ply" "0.25" "0.01"
  python $python_program_possion "./surf.ply" "$output_file" "9"
  end_time=$(date +%s.%N)
  duration=$(echo "$end_time - $start_time" | bc)
  total_time=$(echo "$total_time + $duration" | bc)
  file_count=$((file_count + 1))
done
average_time=$(echo "scale=2; $total_time / $file_count" | bc)
echo "总共处理了 $file_count 个文件"
echo "平均处理时间为 $average_time 秒"