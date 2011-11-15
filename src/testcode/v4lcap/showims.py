fname = "camera_%02d_frame_%03d.raw"
N = 49

idt = np.dtype([('tv_sec', 'u8'), ('tv_usec', 'u8'), ('image', 'u1',
    (480,640,2))])
ims = np.zeros((N,2), dtype=idt)
for i in range(N):
    for c in range(2):
        ims[i,c] = np.fromfile(fname%(c,i), dtype=idt)

