#!/bin/bash

cpp_program="../../build/app/VesselSegment"
inputfolder="/media/wlsdzyzl/DATA1/datasets/pcd/CAS2023/output/xyzr/"
inputpcdfolder="/media/wlsdzyzl/DATA1/datasets/pcd/CAS2023/output/surface"
outputfolder="/media/wlsdzyzl/DATA1/datasets/pcd/CAS2023/output/segment"
# inputfolder="/media/wlsdzyzl/DATA1/datasets/pcd/kipa22/output3/xyzr"
# inputpcdfolder='/media/wlsdzyzl/DATA1/datasets/pcd/kipa22/output3/surface'
# outputfolder="/media/wlsdzyzl/DATA1/datasets/pcd/kipa22/output3/segment"

files=$(find "$inputfolder" -type f)
total_time=0
file_count=0
for file in $files; do
  filename=$(basename "$file" ".xyzr")

  # 指定输出文件路径
  inputpcd="${inputpcdfolder}/${filename}.ply"
  output_file="${outputfolder}/${filename}.ply"
  start_time=$(date +%s.%N)
  echo $cpp_program "$inputpcd" "$file" "$output_file" "2" "0.001" "2" "1"
  # 使用编译好的C++程序处理文件，并指定输出文件路径
  $cpp_program "$inputpcd" "$file" "$output_file" "2" "0.001" "2" "1"
  end_time=$(date +%s.%N)
  duration=$(echo "$end_time - $start_time" | bc)
  total_time=$(echo "$total_time + $duration" | bc)
  file_count=$((file_count + 1))
done
average_time=$(echo "scale=2; $total_time / $file_count" | bc)
echo "总共处理了 $file_count 个文件"
echo "平均处理时间为 $average_time 秒"