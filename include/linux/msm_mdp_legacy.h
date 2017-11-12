#ifndef CONFIG_FB_MSM_NEW
/* HTC: Define custom ioctl started from 200 */
#define MSMFB_OVERLAY_CHANGE_ZORDER_VG_PIPES    _IOW(MSMFB_IOCTL_MAGIC, 200, unsigned int)
#define MSMFB_GET_GAMMA_CURVY _IOWR(MSMFB_IOCTL_MAGIC, 201, struct gamma_curvy)
#endif
