import cv2

# img1 = cv2.imread("1.jpg")
# img2 = cv2.circle(img1, (454,392), 2, (0,0,255), -1)
# cv2.imshow("image1",img1)
#
# img2 = cv2.imread("2.jpg")
# img2 = cv2.circle(img2, (308,500), 2, (0,0,255), -1)
# cv2.imshow("image2",img2)
#
# img4 = cv2.imread("4.jpg")
# img4 = cv2.circle(img4, (447,479), 2, (0,0,255), -1)
# cv2.imshow("image4",img4)
#
# cv2.waitKey(0)
# cv2.destroyAllWindows()

filename = r'matching5.txt'
with open(filename) as f:
    content = f.readlines()
# you may also want to remove whitespace characters like `\n` at the end of each line
content = [x.strip() for x in content]

kp1 = []
kp2 = []
kp3 = []
kp4 = []
kp5 = []
kp6 = []

final2D_list = []

for line in content:
    data = line.split()
    data = data[4:]
    #kp1.append((data[0], data[1]))
    for idx,value in enumerate(data):
        if value == '2':
            kp2.append(((data[0], data[1]),(data[idx + 1], data[idx +2])))
        if value == '3':
            kp3.append(((data[0], data[1]),(data[idx + 1], data[idx +2])))
        if value == '4':
            kp4.append(((data[0], data[1]),(data[idx + 1], data[idx +2])))
        if value == '5':
            kp5.append(((data[0], data[1]),(data[idx + 1], data[idx +2])))
        if value == '6':
            kp6.append(((data[0], data[1]),(data[idx + 1], data[idx +2])))

    # if data[2] == 2:
    #     kp2.append((data[3], data[4]))
    # if data[2] == 3:
    #     kp3.append((data[3], data[4]))
    # if data[2] == 4:
    #     kp4.append((data[3], data[4]))
    # if data[2] == 5:
    #     kp5.append((data[3], data[4]))
    # if data[2] == 6:
    #     kp6.append((data[3], data[4]))
    # try:
    #     if data[5] == 3:
    #         kp3.append((data[6], data[7]))
    #     if data[5] == 4:
    #         kp4.append((data[6], data[7]))
    #     if data[5] == 5:
    #         kp5.append((data[6], data[7]))
    #     if data[5] == 6:
    #         kp6.append((data[6], data[7]))
    # except:
    #     pass


print("kp1 ",kp1)
print("kp2 ",kp2)
print("kp3 ",kp3)
print("kp4 ",kp4)
print("kp5 ",kp5)
print("kp6 ",kp6)

import pickle

with open('matches5_6.data', 'wb') as filehandle:
    # store the data as binary data stream
    pickle.dump(kp6, filehandle)


list1 = []
list2 = []

for pair in kp6:
    list1.append((int(float(pair[0][0])),int(float(pair[0][1]))))
    list2.append((int(float(pair[1][0])),int(float(pair[1][1]))))

# img5 = cv2.imread("5.jpg")
# img6 = cv2.imread("6.jpg")
#
# for i in list1:
#     res1 = cv2.circle(img5, i, 2, (0, 0, 255), -1)
#
# for i in list2:
#     res2 = cv2.circle(img6, i, 2, (0, 0, 255), -1)
#
# cv2.imshow("res1",res1)
# cv2.imshow("res2",res2)
# cv2.waitKey(0)
# cv2.destroyAllWindows()