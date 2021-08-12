#file not currently used. This File is an implementation of kdTree for point cloud storage and segmentation
import math
import numpy as np


class Node:
    def __init__(self, data, point_id):
        self.point = {"R": data[0], "T": data[1]}
        self.point_id = point_id
        self.left_node = None
        self.right_node = None


class KdTree_class:
    def __init__(self):
        self.root = None
        self.kdtree_display_dict = dict()

    def insert_points(self, scan_dataframe, display_output=False):
        """
        :param pcd_dataframe: dataframe containing (R,Theta) coordinates data
        :return:
        """
        for row in scan_dataframe:
            range, angle, point_id = row[0], row[1], np.where(
                scan_dataframe == row)
            point = (range, angle)
            level = 0
            self.root = self.build_kdtree(self.root, level, point, point_id)
        # If display_output is enabled, then display the contents of Kdtree
        if display_output:
            print("Kdtree Build Complete")
            self.display_kdtree(self.root)
            for pair in self.kdtree_display_dict.items():
                print(f"Depth = {pair[0]}, Points = {pair[1]} ")
        return self.root

    def build_kdtree(self, node, depth, point, point_id):
        """
        :param node: Node class object
        :param depth: level0 -Radius, level1-Theta
        :return: root node
        """
        if node is None:
            node = Node(point, point_id)
            return node

        # If current node is empty, then assign the point as root
        current_node = Node(point, point_id)
        # Level should always be within 0-2 range
        depth %= 2
        # levels correspond to (X,Y,Z), check at each level before assigning left/right to the root node
        # self.__dict_key returns X,Y,Z based on the level value. Check function for detailed description
        if node.point[self.__dict_key(depth)] < current_node.point[self.__dict_key(depth)]:
            # If value at level less than current node point, add it as a right node
            node.right_node = self.build_kdtree(
                node.right_node, depth + 1, point, point_id)
        else:
            # If value at level is more than current node point, add it as a left node
            node.left_node = self.build_kdtree(
                node.left_node, depth + 1, point, point_id)
        return node

    def search_elements(self, node, search_point, distance_threshold, depth=0, kdtree_search_results=set()):
        """
        :param node: node of kdtree
        :param search_point: (x,y,z) point
        :param distance_threshold: pcd elements near point
        :param depth: level of the kdtree depth
        :param kdtree_search_results: level of the kdtree depth
        :return: ids which can be considered as near points
        """
        depth %= 2
        current_node = node
        if current_node is not None:
            # If current node is within the distance threshold of search_point
            if(((current_node.point["R"] < search_point[0] + distance_threshold) and (current_node.point["R"] > search_point[0] - distance_threshold)) and
               ((current_node.point["T"] < search_point[1] + distance_threshold) and (current_node.point["T"] > search_point[1] - distance_threshold))):
                # Calculate the distance of search point from current node
                point_distance = math.sqrt(math.pow(current_node.point["R"], 2) + math.pow(
                    search_point[0], 2)-2*current_node.point["R"]*search_point[0]*math.cos(current_node.point["T"]-search_point[1]))
                if point_distance <= distance_threshold:
                    kdtree_search_results.add(current_node.point_id)
                else:
                    pass
            # Iterate recursively
            # if current_node.point[self.__dict_key(depth)] < search_point[depth]:
            if current_node.point[self.__dict_key(depth)] < search_point[depth] + distance_threshold:
                self.search_elements(current_node.right_node, search_point, distance_threshold,
                                     depth+1, kdtree_search_results)

            # else:
            if current_node.point[self.__dict_key(depth)] > search_point[depth] - distance_threshold:
                self.search_elements(current_node.left_node, search_point, distance_threshold,
                                     depth+1, kdtree_search_results)
            return kdtree_search_results

    def display_kdtree(self, node, depth=0):
        """
        updates the self.kdtree_dict with the points are corresponding depth
        :param node: root node
        :param depth: indicates the depth of Kdtree
        """
        current_node = node
        try:
            # If there are values already present, append the list with the point.
            self.kdtree_display_dict[depth].extend([(current_node.point["R"],
                                                     current_node.point["T"])])
        except KeyError:
            # If there are no values at the level, add value as first point
            self.kdtree_display_dict[depth] = [(current_node.point["R"],
                                                current_node.point["T"])]
        # Run the recursion until a function hits the empty node
        if current_node is not None:
            # Check
            if current_node.left_node is not None:
                left_node = current_node.left_node
                # increment the value of depth
                depth += 1
                # at every node, call the recursive function
                self.display_kdtree(left_node, depth)

            if current_node.right_node is not None:
                right_node = current_node.right_node
                # increment the value of depth
                depth += 1
                # at every node, call the recursive function
                self.display_kdtree(right_node, depth)

    @staticmethod
    def __dict_key(number):
        """
        returns R,Theta based on number
        :param number: 0,1
        :return: R,Theta
        """
        try:
            key_dict = {0: "R", 1: "T"}
            return str(key_dict[number])
        except KeyError:
            raise Exception(f"Incorrect Level({number}) Assignment.")
