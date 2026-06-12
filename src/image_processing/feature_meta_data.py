class FeatureMetaData(object):
    """
    Stores the information required for convenient feature access.
    """
    def __init__(self):
        self.id = None           # int
        self.response = None     # float
        self.lifetime = None     # int
        self.cam0_point = None   # vec2
        self.cam1_point = None   # vec2