/* 
 * File:   TMC2209.h
 * Author: Brian Schmalz
 *
 * Created on June 5, 2020, 10:24 PM
 */

#ifndef TMC2209_H
#define	TMC2209_H

/* Register addresses */
#define GCONF         0x00
#define GSTSAT        0x01
#define IFCNT         0x02
#define SLAVECONF     0x03
#define OTP_PROG      0x04
#define OTP_READ      0x05
#define IOIN          0x06
#define FACTORY_CONF  0x07

#define IHOLD_RUN     0x10
#define TPOWERDOWN    0x11
#define TSTEP         0x12
#define TPWMTHRS      0x13
#define VACTUAL       0x22

#define TCOOLTHRS     0x14
#define SGTHRS        0x40
#define SG_RESULT     0x41
#define COOLCONF      0x42

#define MSCNT         0x6A
#define MSCURACT      0x6B

#define CHOPCONF      0x6C
#define DRV_STATUS    0x6F
#define PWMCONF       0x70
#define PWM_SCALE     0x71
#define PWM_AUTO      0x72 

#endif	/* TMC2209_H */

