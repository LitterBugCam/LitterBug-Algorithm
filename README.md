FYI: OpenCV is only used for image aquisition and preprocessing steps. The core routines are written in  C/C++. 
The readability of the code will be improved soon !

## LitterDetection

This a program for Litter detection events in video surveillance. An object (of any type) is detected secondes after
its owner has left it in the camera FOV. The algorithm is an improved version of the algorithm  we first proposed  in :

[**Ilias, D. A. H. I., et al. "An edge-based method for effective abandoned luggage detection in complex surveillance videos." Computer Vision and Image Understanding 158 (2017): 141-151.**](https://www.sciencedirect.com/science/article/pii/S1077314217300243)

# LitterBug Project

This program is part of the LitterBug project, a battery powered camera system for litter prevention. 

For more information, please visit our website https://litterbug.cam/

# Prerequisites
1. OpenCV

# Detection example
check this video !
<div align="center">
  <a href="https://www.youtube.com/watch?v=7AQPiU4QIy0"><img src="https://img.youtube.com/vi/7AQPiU4QIy0/0.jpg" alt="Detection example"></a>
</div>

# Configuration
  
  You can modify the Parameters.txt file to adapt the algorithm to the scene you are running the algorithm on and to the type of abandoned object you are trying to detect.
  
# Setup

Just run make command in the source code folder

# Usage
./**Litter_Detect** videopath 

# Contact 

For any suggestions or comment, please feel free to contact Ilias Dahi (ilias@evercam.io)
