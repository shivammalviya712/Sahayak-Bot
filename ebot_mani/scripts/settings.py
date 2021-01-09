"""
Parameters' values

Author: eYRC_SB_363
"""



class Config(object):
    """
    - Store the values of the hyperparamter of a particular object
    """
    def __init__(self, **kwargs):
        self.__dict__.update(kwargs)


class Settings(object):
    """
    Store the values of all the hyperparamters
    """
    def __init__(self):
        self.camera = self.init_camera()
        

    def init_filter(self):
        """
        Set the filters' paramters
        """
        voxel_filter = Config(
            leaf_size=0.003,
        )
        # Homogenous coordinate
        cropbox_filter = Config(
            translation=(0, 0, 0),
            rotation=(0, 0, 0),
            ranges=(
                -0.5, -0.5, 0.1, 1,
                0.5, 0.5, 1.1, 1       
            )
        )
        ransac_filter = Config(
            threshold=0.01
        )
        statistical_outlier_filter = Config(
            mean_k=50,
            std_dev_mul_thresh=1.0
        )
        euclidean_cluster_filter = Config(
            cluster_tolerance=0.02,
            max_cluster_size=25000,
            min_cluster_size=800
        )
        
        return Config(
            voxel_filter=voxel_filter,
            cropbox_filter=cropbox_filter,
            ransac_filter=ransac_filter,
            statistical_outlier_filter=statistical_outlier_filter,
            euclidean_cluster_filter=euclidean_cluster_filter
        )


    def init_pointcloud(self):
        """
        Set pointcloud's parameters
        """
        pointcloud = Config(
            filters=self.init_filter(),
            save_pc_enable=True,
            analysis_enable=False
        )

        return pointcloud


    def init_camera(self):
        """
        Set the camera's paramters
        """
        poincloud = self.init_pointcloud()
        
        return Config(
            pointcloud=poincloud
        )
