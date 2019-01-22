# -*- coding: utf-8 -*- 

#------------------------------------ Imports ----------------------------------#

# Import python imaging libs
from PIL import Image
from PIL import ImageDraw
from PIL import ImageFont
from PIL import ImageFilter
import numpy as np
import cv2

# Import operating system lib
import os

# Import random generator
from random import randint

#------------------------------------ Cleanup ----------------------------------#
                    
def Cleanup():    
    # Delete ds_store file
    if os.path.isfile(font_dir + '.DS_Store'):
        os.unlink(font_dir + '.DS_Store')
    
    # Delete all files from output directory
    for file in os.listdir(out_dir):
        file_path = os.path.join(out_dir, file)
        if os.path.isfile(file_path):
            os.unlink(file_path)
    return

#------------------------------ Generate Characters ----------------------------#

def GenerateCharacters():
    # Counter
    k = 1
    # Process the font files
    for dirname, dirnames, filenames in os.walk(font_dir):
        # For each font do
        for filename in filenames:
            # Get font full file path
            font_resource_file = os.path.join(dirname, filename)
            
            # Character counter for naming folders
            char_counter = 0
            # For each character do
            for char in characters:
                # For each font size do
                for font_size in font_sizes:
                    if font_size > 0:
                        # For each background color do
                        for background_color in background_colors:
                            # Convert the character into unicode
                            character = unicode(char, 'utf-8')
                            #character = str(char)
            
                            # Create character image : 
                            # Grayscale, image size, background color
                            char_image = Image.new('L', (image_size, image_size), background_color + randint(0,bkg_color_increment))
            
                            # Draw character image
                            draw = ImageDraw.Draw(char_image)
            
                            # Specify font : Resource file, font size
                            font = ImageFont.truetype(font_resource_file, font_size)
            
                            # Get character width and height
                            (font_width, font_height) = font.getsize(character)
            
                            # Calculate x position
                            x = (image_size - font_width)/2
            
                            # Calculate y position
                            y = (image_size - font_height)/2 - 2
            
                            # Draw text : Position, String, 
                            # Options = Fill color, Font
                            color = randint(230,240) + background_color/150*35
                            if color > 255:
                                color = 255
                            draw.text((x, y), character, color, font=font)
                            char_image = np.array(char_image)
                            kernel = np.ones((2,2),np.uint8)
                            char_image = cv2.dilate(char_image,kernel,iterations=1)
                            char_image = cv2.GaussianBlur(char_image, (3,3), 0)
                            #char_image_cpy = char_image.copy()
                            #char_image = cv2.add(char_image, (char_image_cpy*0.1).astype(np.uint8))
                            char_image = Image.fromarray(char_image)
                            
                            # Final directory path
                            if char_counter < 10:
                                char_counter_str = '0' + str(char_counter)
                            else:
                                char_counter_str = str(char_counter)
                            dir_path = out_dir + char_counter_str + '.' + character + '/'
                            
                            # Final file name                    
                            file_name = dir_path + str(batch) + '-' + str(k) + '_' + filename.split('.')[0] + \
                            '_fs_' + str(font_size) + \
                            '_bc_' + str(background_color) + \
                            '_' + character + '.png'
                            
                            # Save image
                            if not os.path.exists(dir_path):
                                os.makedirs(dir_path)
                            char_image.save(file_name)
                    
                            # Print character file name
                            print(file_name);
                    
                            # Increment counter
                            k = k + 1
                            #break
                    #break
                #break
                char_counter += 1
            #break
        #break
    return

#---------------------------------- Input and Output ---------------------------#

if not os.path.exists('generated_char'):
    os.makedirs('generated_char')

# Directory containing fonts
font_dir = 'font/'

# Output
out_dir = 'generated_char/'

#------------------------------------ Characters -------------------------------#

batch = 1

symbols = ['~','`','!','@','#','$','%','^','&','*','(',')','_','-','=','+',']','}','[','{',':',';','?','>','<']


numbers = ['0','1','2','3','4','5','6','7','8','9']


small_letters = [] #['a','b','c','d','e','f','g','h','i','j','k','l','m','n','o','p','q','r','s','t','u','v','w','x','y','z']

capital_letters = ['A','B','C','D','E','F','G','H','I','J','K','L','M','N','O','P','Q','R','S','T','U','V','W','X','Y','Z']

# Select characters
characters = capital_letters + small_letters + numbers + symbols

#------------------------------------- Colors ----------------------------------#

# Background color (0-255)

bkg_color_increment = 5

background_colors = [30,35,40,45,50,55,60,65,70,75,80,85,90,95,100,105,110,115,120,125,130,135,140,145]
        
#-------------------------------------- Sizes ----------------------------------#

# Character sizes
font_sizes = [23,26,28]
        
# Image size
image_size = 30

#-------------------------------------- Main -----------------------------------#

# Do cleanup
#Cleanup()

# Generate characters
GenerateCharacters()
