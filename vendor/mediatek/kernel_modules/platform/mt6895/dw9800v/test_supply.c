
#include <stdint.h>
#include <assert.h>
#include <errno.h>
#include <string.h>
#include <stdio.h>
#define EPROBE_DEFER 517
#define ERR_PTR(e) ((void *)(intptr_t)(e))
#define PTR_ERR(p) ((intptr_t)(p))
#define IS_ERR(p) ((uintptr_t)(p)>=(uintptr_t)-4095)
struct device {int id;}; struct regulator {int id;};
static struct regulator actual={1};static int fault,lookups;
static struct regulator *devm_regulator_get_optional(struct device *d,const char *n){
 assert(d&&d->id==1&&!strcmp(n,"fan53870-l6"));lookups++;
 return fault?ERR_PTR(fault):&actual;
}
static struct regulator *dw9800v_get_motor_supply(struct device *dev)
{
	struct regulator *supply;

	supply = devm_regulator_get_optional(dev, "fan53870-l6");
	if (IS_ERR(supply) && PTR_ERR(supply) == -ENODEV)
		return ERR_PTR(-EPROBE_DEFER);
	return supply;
}
int main(void){
 struct device d={1};int errors[]={-ENODEV,-EPROBE_DEFER,-ENOMEM,-EIO,-EBUSY,-EACCES};
 for(int i=0;i<6;i++){fault=errors[i];assert(PTR_ERR(dw9800v_get_motor_supply(&d))==(fault==-ENODEV?-EPROBE_DEFER:fault));}
 /* Supplier registration must allow the previously deferred consumer to bind. */
 for(int i=0;i<100;i++){fault=-ENODEV;assert(PTR_ERR(dw9800v_get_motor_supply(&d))==-EPROBE_DEFER);}
 fault=0;assert(dw9800v_get_motor_supply(&d)==&actual);assert(lookups==107);
 puts("PASS: absent/late/present supplier, preserved errors, no dummy acceptance");
}
