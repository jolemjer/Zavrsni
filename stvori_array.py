import cv2
import numpy as np
from matplotlib import pyplot as plt
import rospy
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Point

# Zavrsni projekt
# Egzaktna simulacija automnog istraživanja prostora pomoću višerobotskog sustava
# napravio: Josip Spudic, josip.spudic@fer.hr
# mentori: Stjepan Bogdan i Ana Batinovic

# uzimanje vrijednosti prametara
# resolution =  rospy.get_param('/resolution') # resolution in meters, u .world je default
# scale_percent = rospy.get_param('/scale_percent') # percent of original size

# ucitaj
img_name = rospy.get_param('/image')                # getting a image name from yaml file 
img = cv2.imread(img_name, cv2.IMREAD_GRAYSCALE)    

width = rospy.get_param('/width')
height = rospy.get_param('/height')
dim = (width, height)

# smanjujem rezoluciju mape jer je presporo
img= cv2.resize(img, dim, interpolation = cv2.INTER_AREA)

img_flatten = img.ravel()


# with histogram you'll get border occupied and free space
# not important, for most cases (low image values - free space, high image values - occupied space)

# def histogram():
# # Display the image and histograms.
#     plt.figure(figsize = [18, 4])
#     plt.subplot(121); plt.imshow(img, cmap='gray'); plt.title('Original Image')
#     plt.subplot(122); plt.hist(img_flatten, 50, [0, 256]); plt.xlim([0, 256]); plt.title('50 Bins histogram');
#     plt.show()
#histogram()


# iz histograma odredimo granicu sive i crne, vjerojatno će isto biti 220
granica1 = rospy.get_param('/granica')
granica2 = 50  # ne ucitavam iz ros_param taj podatak
               # ako je potrebno onda stavi

# probabilities are in the range [0,100].  Unknown is -1.
for i in range(height):
    for j in range(width):
        if img[i][j] > granica1:
            img[i][j] = 0
        elif img[i][j] < granica2:
            img[i][j] = 100
        else:
            img[i][j] = -1

# iz slika napravi mapu
# ne mozemo poslati 2D matricu jer nav_msgs/OccupancyGrid prima samo listu vrijednosti
map_znamo = img.flatten().view(np.int8)

def talker():
    pub = rospy.Publisher('all_grid', OccupancyGrid, queue_size=10)
    rospy.init_node('publish_grid', anonymous=True)

    topic = OccupancyGrid()
    
    topic.info.resolution = 1 # stavili smo 1 da mogu vidjeti robota s Odometryom
    topic.info.width = width
    topic.info.height = height
    p = Point(0,0,0)
    topic.info.origin.position = p
    
    # mapa se tijekom izvodenja nece mijenjati
    # zato saljemo stalno istu vrijednost
    # trebalo se drugacije napraviti jer mapa od slike je uvijek ista i onda se uvijek ista vrijednost publisha
    # napravio tako da imam neki rate u otkrivanju prostora s robota
    # svaki put kad dobi mapu robot ce izgraditi mapu koju je on otkrio
    while not rospy.is_shutdown():
        topic.data = map_znamo
        pub.publish(topic)
        rospy.sleep(0.1) # napravio sam malo vise da ceka 
        
talker()