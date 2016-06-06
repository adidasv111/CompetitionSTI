import numpy as np
a = (1,4)
b = (3,15)
ar = [a,b]
ar = np.array(ar, dtype="int")
print(ar)


print(ar[np.argsort(ar[:, 1]), :])
print(ar[0])
