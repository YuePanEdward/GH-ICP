#data path
target_point_cloud_path=/media/edward/BackupPlus/Data/ETH_registration_TLS/arch/PointCloud_pcd/s1.pcd;
source_point_cloud_path=/media/edward/BackupPlus/Data/ETH_registration_TLS/arch/PointCloud_pcd/s2.pcd;
output_point_cloud_path=/media/edward/BackupPlus/Data/ETH_registration_TLS/arch/PointCloud_pcd/reg_s2.pcd;

#parameters
using_feature=B;
appro_overlap_ratio=0.8;
downsample_resolution=0.2;
neighborhood_radius=0.6;
curvature_non_max_radius=1.5;

./bin/ghicp ${target_point_cloud_path} ${source_point_cloud_path} ${output_point_cloud_path} ${using_feature}\
${downsample_resolution} ${neighborhood_radius} ${curvature_non_max_radius} ${appro_overlap_ratio}
