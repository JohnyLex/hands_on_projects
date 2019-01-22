import os
import random
import shutil

val_portion = 0.2

if os.path.exists('generated_char_split'):
    shutil.rmtree('generated_char_split')
os.makedirs('generated_char_split')

dirs = os.listdir('generated_char')
dirs.sort()
    
for d in dirs:
    files = os.listdir('generated_char/' + d)
    num = len(files)
    valid_idx = random.sample(range(num), int(num*val_portion))
    train_idx = [i for i in range(num) if i not in valid_idx]
    
    train_files = [files[i] for i in train_idx]
    valid_files = [files[i] for i in valid_idx]
    
    os.makedirs('generated_char_split/train/' + d)
    os.makedirs('generated_char_split/valid/' + d)
    
    for f in train_files:
        src = 'generated_char/' + d + '/' + f
        dst = 'generated_char_split/train/' + d + '/' + f
        shutil.copyfile(src,dst)
    
    for f in valid_files:
        src = 'generated_char/' + d + '/' + f
        dst = 'generated_char_split/valid/' + d + '/' + f
        shutil.copyfile(src,dst)
