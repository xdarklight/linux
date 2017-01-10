#ifndef USB_HOST_PLATFORM_ROOTHUB_H
#define USB_HOST_PLATFORM_ROOTHUB_H

struct platform_roothub;

struct platform_roothub *platform_roothub_init(struct device *dev);
int platform_roothub_exit(struct platform_roothub *plat_roothub);

int platform_roothub_power_on(struct platform_roothub *plat_roothub);
int platform_roothub_power_off(struct platform_roothub *plat_roothub);

#endif /* USB_HOST_PLATFORM_ROOTHUB_H */
