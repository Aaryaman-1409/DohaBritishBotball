#include <kipr/botball.h>
int main()
{
    printf("Hello World\n");
    enable_servos();
    set_servo_position(3,1400);//claw open 
    msleep(500);
    set_servo_position(0,1850);// claw down
    msleep(500);
    disable_servos();
    clear_motor_position_counter(0);
    clear_motor_position_counter(3);
    get_motor_position_counter(0);
    get_motor_position_counter(3);
    int distance = 5500;
    while (gmpc(0) < distance)
    {
        motor(0,100);
        motor(3,100);
    }

    ao();
    enable_servos();
    set_servo_position(3,1850);//close claw
    msleep(500);
    //set_servo_position(0,150); THIS SETS THE ARM UP TO NOT KNOCK THE CUBES OFF
    //msleep(500);
    disable_servos();
    motor(3,75);//turning 
    motor(0,-75);
    msleep(2000);
    ao();
    enable_servos();
    set_servo_position(0,1950);//claw down 
    msleep(1000);
    set_servo_position(3,1300);//open claw
    msleep(500);
    set_servo_position(0,1400);//claw up
    cmpc(0);
    while (gmpc(0) >= -1800)
    {
        motor(0,-70);
        motor(3,-70);
    } 
    motor(0,-100);
    motor(3,0);
    msleep(4000);

    //set_servo_position(0,150);//arm up 
    //msleep(500);
   // motor(0,75);// turn back to pile 
    //motor(3,-75);
    //msleep(1995);// OTHERWISE IT TURNS TOO FAR
    //set_servo_position(0,1150); // arm down 
    //msleep(500);
    //set_servo_position(3,1850);//close claw
    //msleep(1000);
    //set_servo_position(0,500);//arm up
    //msleep(1000);
    //motor(0,-75);//turn back 
    //motor(3,75);
    //msleep(2000);
    //set_servo_position(0,900);//arm down
    //msleep(500);
    //set_servo_position(3,1500);//claw open 
    //msleep(500);
        
    ao();
    disable_servos();
    
        
        
        
    
        
    
        
    
    return 0;
    
    
}
