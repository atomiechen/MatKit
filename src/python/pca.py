import numpy as np
from sklearn.decomposition import PCA


from toolkit import filemanager


filename_h = "imu_h.csv"
filename_v = "imu_v.csv"

def get_model(filename):
	X = []
	with open(filename, 'r') as fin:
		for line in fin:
			data_parse, _, _ = filemanager.parse_line(line, 4, ',')
			X.append(data_parse)
	X = np.array(X)
	pca = PCA(n_components=1)
	pca.fit(X)
	return pca

pca_h = get_model(filename_h)
pca_v = get_model(filename_v)

