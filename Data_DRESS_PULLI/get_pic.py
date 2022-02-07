import csv
import os
import shutil



target_dir_train = 'train/'
target_dir_test = 'demo_images/'
class1 =   "Pullover" #Change to wanted class 1
class2 =  "Dress" #change to wanted class 2


os.mkdir(target_dir_train)
os.mkdir(os.path.join(target_dir_train, class1))
os.mkdir(os.path.join(target_dir_train, class2))


os.mkdir(target_dir_test)
os.mkdir(os.path.join(target_dir_test, class1))
os.mkdir(os.path.join(target_dir_test, class2))



# get the image 

with open("index.csv") as csv_file:
    csv_reader = csv.reader(csv_file, delimiter=',')
    line_count = 0
    for row in csv_reader:
        if line_count == 0:
            print(f'Column names are {", ".join(row)}')
            line_count += 1
        elif (line_count < 100):
            if (row[0] == class1):
                shutil.move(row[1], os.path.join(target_dir_test, class1))
            elif (row[0] == class2):
                shutil.move(row[1], os.path.join(target_dir_test, class2))
            line_count += 1
        
        else:
            if (row[0] == class1):
                shutil.move(row[1], os.path.join(target_dir_train, class1))
            elif (row[0] == class2):
                shutil.move(row[1], os.path.join(target_dir_train, class2))
            line_count += 1
        




    print(f'Processed {line_count} lines.')