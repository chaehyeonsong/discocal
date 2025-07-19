import numpy as np
from matplotlib import pyplot as plt
from matplotlib.patches import Ellipse
from scipy.interpolate import LinearNDInterpolator
import cv2


def read_matrix(data_path, ignore_cols=[],has_header=False):
    # data_path= "../data/exp_0326/circle_1/left/gradient.txt"
    with open(data_path, 'r') as f:
        if has_header:
            f.readline()
        lines = f.readlines()
    len_col=len(lines[0].split('\t'))
    data= np.zeros([len(lines),len_col])

    for i in range(len(lines)):
        line = lines[i][:-1]
        tokens = line.split('\t')
        for j in range(len_col):
            data[i,j]=float(tokens[j])

    ignore_cols= sorted(ignore_cols, reverse=True)
    for col in ignore_cols:
        data = np.delete(data, col, axis=1)

    return data

def gpr(params,ignore_scenes=[], measure_unc=True, n_samples=None):
    root_dir = params['results_dir']

    n_d =params['n_d']
    fx = params['fx']
    fy = params['fy']
    cx = params['cx']
    cy = params['cy']
    skew =  params['skew']
    d1 = params['d1']
    d2 = params['d2']
    d3 =  params['d3']

    measurements = read_matrix(root_dir+"measurements.txt",has_header=True)
    Jacob_original = read_matrix(root_dir+"jacob.txt")
    mask =np.ones(measurements.shape[0])>0
    for scene_i in ignore_scenes:
        mask[12*scene_i:12*(scene_i+1)]=False

    Jacob_mask = np.stack([mask,mask]).T.flatten()
    Jacob_train = Jacob_original[Jacob_mask]
    Jacob_col_mask = np.sum(Jacob_train!=0,axis=0)>0
    Jacob_train = Jacob_train[:,Jacob_col_mask]
    measurements = measurements[mask]

    # Jacob_train = Jacob_original
    Y_train = measurements[:,:2]

    Cov_y = measurements[:,2:]
    Cov_Y  = np.zeros([Cov_y.shape[0]*2,Cov_y.shape[0]*2])
    for i in range(Cov_y.shape[0]):
        Cov_Y[2*i,2*i]= Cov_y[i,0]
        Cov_Y[2*i+1,2*i]= Cov_y[i,1]
        Cov_Y[2*i,2*i+1]= Cov_y[i,1]
        Cov_Y[2*i+1,2*i+1]= Cov_y[i,2]

    param_range = 5+n_d
    s_range= params['width']/params['fx']/2*pow(1-params['d1'],2)
    if n_samples is None:
        n_samples = int(s_range*20)
    xs = np.linspace(-s_range,s_range,n_samples)
    ys = np.linspace(-s_range,s_range,n_samples)
    xv, yv = np.meshgrid(xs,ys)
    xyd = np.stack([xv.flatten(), yv.flatten()]).T

    camera_matrix = np.identity(3)
    dist_coeff = np.array([d1, d2, 0, 0, d3])
    xyn= cv2.undistortPoints(xyd, camera_matrix, dist_coeff, None).squeeze()
    X_test =[]
    X_test_d = []
    for i in range(len(xyn)):
        xn, yn = xyn[i]
        s1 = xn*xn+yn*yn+0.0
        s2 = s1*s1
        s3 = s2*s1
        k = 1 + d1*s1+d2*s2+d3*s3
        xd= k*xn
        yd = k*yn
        error = (xd-xyd[i,0])**2+(yd-xyd[i,1])**2
        if error<0.01:
            X_test.append([xn,yn])
            X_test_d.append([fx*xyd[i,0]+skew*xyd[i,1]+cx, fy*xyd[i,1]+cy])

    X_test = np.array(X_test,dtype=np.float64)
    X_test_d = np.array(X_test_d,dtype=np.float64)


    n_data = X_test.shape[0]
    Jacob_test  = np.zeros([n_data*2,8],dtype=np.float64)
    mean_prediction = []
    for i in range(n_data):
        xn, yn = X_test[i]
        s1 = xn*xn+yn*yn+0.0
        s2 = s1*s1
        s3 = s2*s1
        k = 1 + d1*s1+d2*s2+d3*s3
        temp_u = fx*xn+skew*yn
        temp_v = fy*yn
        Jacob_test[2*i] = np.array([k*xn, 0, 1, 0 , k*yn, s1*temp_u, s2*temp_u, s3*temp_u])
        Jacob_test[2*i+1] = np.array([0 , k*yn, 0, 1 , 0, s1*temp_v, s2*temp_v, s3*temp_v])
        mean_prediction.append([fx*k*xn+skew*k*yn+cx, fy*k*yn+cy])

    mean_prediction = np.array(mean_prediction,dtype=np.float64)

    # X = Jacob_train[:,:param_range]
    X = Jacob_train
    x = Jacob_test[:,:param_range]
    info_m = np.linalg.inv(Cov_Y)
    if measure_unc:
        A_info = X.T@info_m@X
    else:
        A_info = X.T@X

    A_info = A_info
    A_inv = np.linalg.inv(A_info)[:param_range,:param_range]
    var_prediction = x@A_inv@x.T

    return Y_train, mean_prediction, var_prediction, A_inv, A_info

def cov2unc(cov):
    unc = 2*np.sqrt(np.trace(cov)/2)
    return unc
    
def cov_geo(cov):
    assert ~np.any(np.isnan(cov))
    assert np.all((cov- cov.T)<1e-8)
    evd = np.linalg.eigh(cov)
    assert np.all(evd[0]>0)
    m0_index= np.argmax(evd[0])
    m1_index = 1- m0_index
    m0 = max(0, evd[0][m0_index])
    m1 = max(0, evd[0][m1_index])
    v0 = evd[1][:,m0_index]
    # v1 = evd[1][:,m1_index]
    angle = int(np.arctan2(v0[1],v0[0])*180/np.pi)
    axis = 2*np.sqrt(np.array([m0,m1]))
    return axis, angle


def evaluate(params):
    width = params['width']
    height = params['height']
    params['n_sample']=25

    ignore_scenes =[]
    measurements, mean_prediction, var_prediction,A_inv,A_info = gpr(params, ignore_scenes=ignore_scenes,measure_unc=True)

    x = mean_prediction[:,0]
    y = mean_prediction[:,1]
    z =[]
    scale = 1
    for i in range(mean_prediction.shape[0]):
        var_i = var_prediction[2*i:2*i+2,2*i:2*i+2]
        # unc_i = np.power(np.linalg.det(var_i),0.25)*2
        unc_i = cov2unc(var_i)
        z.append(unc_i) #rms
    z = np.array(z)
    interp = LinearNDInterpolator(list(zip(x, y)), z)

    X = np.linspace(0, width-1, width)
    Y = np.linspace(0, height-1, height)
    X, Y = np.meshgrid(X, Y)  # 2D grid for interpolation
    Z = interp(X, Y)*scale
    Z_valid = Z[~np.isnan(Z)]
    n80 = int(len(Z_valid)*0.8)
    vmax = max(2,np.sort(Z_valid)[n80])

    ratio = height/width
    fig, axes = plt.subplots(nrows=1, ncols=1, figsize=(10,10*ratio-1))
    pcm = axes.imshow(Z,vmin=0.0, vmax= max(vmax,2))
    cb = fig.colorbar(pcm, ax=axes, shrink=1)

    # max_cov= 300
    # scale = 10
    # measurements, mean_prediction, var_prediction,A_inv,A_info = gpr(params, ignore_scenes=ignore_scenes,measure_unc=True, n_samples=10)
    # for i in range(mean_prediction.shape[0]):
    #     axis, angle= cov_geo(var_prediction[2*i:2*i+2,2*i:2*i+2])
    #     e_x = mean_prediction[i,0]
    #     e_y = mean_prediction[i,1]
    #     if (e_x >0 and e_x < width) and (e_y>0 and e_y < height):
    #         major_ax = min(int(axis[0]*scale*2), max_cov)
    #         minor_ax = min(int(axis[1]*scale*2),max_cov)
    #         ellipse = Ellipse(xy=(e_x, e_y), width=major_ax, height=minor_ax, angle=angle, edgecolor='orangered', fc='None', lw=2)
    #         axes.plot( e_x, e_y, 'o', color = 'orangered', markerfacecolor= 'orangered' ,markersize=4)
    #         # axes.scatter(mean_prediction[:,0],mean_prediction[:,1], label="Prediction", s=10, marker='x')
    #         axes.add_patch(ellipse)
    # axes.set_xlabel(f"Ellipse scale:{scale}")

    axes.scatter(measurements[:,0],measurements[:,1],  s=30, edgecolors='w', label="Measurements")
    axes.legend()
    axes.set_title("Calibration uncertainty per pixels")



    ## visualization
    cv2.imwrite(params['results_dir']+"./unc_map.tif", Z)
    img_name = "calibration_uncertainty"
    img_save_path = params["results_dir"]+img_name+".png"
    fig.savefig(img_save_path,bbox_inches='tight', pad_inches=0.1)

    output_img = cv2.imread(img_save_path)
    cv2.imshow(img_name,output_img)
    print("Press any key to close the window")
    key = cv2.waitKey(0)
    cv2.destroyAllWindows()

    

