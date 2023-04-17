# Defines a 2D detection result.
#
# This is similar to a 2D classification, but includes position information,
#   allowing a classification result for a specific crop or image point to
#   to be located in the larger image.

Header header

# Class probabilities
int64 id
# string class_name
float64 confidence_score

# 2D bounding box surrounding the object.
Bbox2D bbox
