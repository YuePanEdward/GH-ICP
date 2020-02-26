#data path
target_point_cloud_path=/media/edward/BackupPlus/Data/ETH_registration_TLS/arch/PointCloud_pcd/s1.pcd;
source_point_cloud_path=/media/edward/BackupPlus/Data/ETH_registration_TLS/arch/PointCloud_pcd/s2.pcd;
output_point_cloud_path=/media/edward/BackupPlus/Data/ETH_registration_TLS/arch/PointCloud_pcd/reg_s2.pcd;

#parameters
using_feature=B;
corres_estimation_method=K;

downsample_resolution=0.2;
neighborhood_radius=0.6;
curvature_non_max_radius=1.5;
weight_adjustment_ratio=1.1;
weight_adjustment_step=0.1;

appro_overlap_ratio=0.8;

./bin/ghicp ${target_point_cloud_path} ${source_point_cloud_path} ${output_point_cloud_path} \
${using_feature} ${corres_estimation_method} \
${downsample_resolution} ${neighborhood_radius} ${curvature_non_max_radius} \
${weight_adjustment_ratio} ${weight_adjustment_step} ${appro_overlap_ratio}
