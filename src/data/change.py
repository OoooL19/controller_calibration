f = open('landmark_small.txt', 'r')
# file = f.read()
# print(file)
# f.close()
for i in f:
	x = float(i)
	print(x/1000)