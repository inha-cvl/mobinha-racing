import geopandas as gpd


class NGIIParser:
    def __init__(self,
        a1_path,
        a2_path,
        b2_path):

        self.a1_node = self.parse_a1_node(a1_path)
        self.a2_link = self.parse_a2_link(a2_path)
        self.b2_surfacelinemark = self.parse_b2_surfacelinemark(b2_path)
    def parse_a1_node(self, path):
        shape_data = gpd.read_file(path)
        a1_node = []
        for index, data in shape_data.iterrows():
            a1_node.append(data)
        return a1_node

    def parse_a2_link(self, path):
        shape_data = gpd.read_file(path)
        a2_link = []
        for index, data in shape_data.iterrows():
            a2_link.append(data)
        return a2_link

    def parse_b2_surfacelinemark(self, path):
        shape_data = gpd.read_file(path)
        b2_surfacelinemark = []
        for index, data in shape_data.iterrows():
            b2_surfacelinemark.append(data)
        return b2_surfacelinemark