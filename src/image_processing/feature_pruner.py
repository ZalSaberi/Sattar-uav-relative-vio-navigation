class FeaturePruner:
    def __init__(self, grid_max_feature_num):
        """
        grid_max_feature_num: maximum number of features per grid cell
        """
        self.grid_max_feature_num = grid_max_feature_num

    def prune_features(self):
        """
        Removes some features from a grid cell when there are too many,
        so that the number of features in each cell stays bounded.
        """
        for i, features in enumerate(self.curr_features):
            # Continue if the number of features in this grid does
            # not exceed the upper bound.
            if len(features) <= self.config.grid_max_feature_num:
                continue
            self.curr_features[i] = sorted(features, key=lambda x:x.lifetime, 
                reverse=True)[:self.config.grid_max_feature_num]   