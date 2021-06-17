//CSCI 5611 Vector 3 Library 

public class Vec3 {
  public float x, y, z;
  
  public Vec3(float x, float y, float z){
    this.x = x;
    this.y = y;
    this.z = z;
  }
  
  public String toString(){
    return "(" + x+ ", " + y + ", " + z + ")";
  }
  
  public float length(){
    return (float)Math.sqrt(Math.pow(x,2.0) + Math.pow(y,2.0)+ Math.pow(z,2.0));
  }
  
  public float lengthSqr(){
    return (float)(Math.pow(x,2.0) + Math.pow(y,2.0) + Math.pow(z,2.0));
  }
  
  public Vec3 plus(Vec3 rhs){
    float x_sum, y_sum, z_sum;
    x_sum = this.x + rhs.x;
    y_sum = this.y + rhs.y;
    z_sum = this.z + rhs.z;
    return new Vec3(x_sum,y_sum, z_sum);
  }
  
  public void add(Vec3 rhs){
    this.x += rhs.x;
    this.y += rhs.y;
    this.z += rhs.z;
  }
  
  public Vec3 minus(Vec3 rhs){
    float x_sub, y_sub, z_sub;
    x_sub = this.x - rhs.x;
    y_sub = this.y - rhs.y;
    z_sub = this.z - rhs.z;
    return new Vec3(x_sub,y_sub,z_sub);
  }
  
  public void subtract(Vec3 rhs){
    this.x -= rhs.x;
    this.y -= rhs.y;
    this.z -= rhs.z;
  }
  
  public Vec3 times(float rhs){
    float x_mul, y_mul, z_mul;
    x_mul = this.x * rhs;
    y_mul = this.y * rhs;
    z_mul = this.z * rhs;
    return new Vec3(x_mul,y_mul,z_mul);
  }
  
  public void mul(float rhs){
    this.x *= rhs;
    this.y *= rhs;
    this.z *= rhs;
  }
  
  public void normalize(){
    float curL = length();
    this.x /= curL;
    this.y /= curL;
    this.z /= curL;
  }
  
  public Vec3 normalized(){
    float x_norm, y_norm, z_norm;
    float curL = this.length();
    x_norm = this.x / curL;
    y_norm = this.y / curL;
    z_norm = this.z /curL;
    return new Vec3(x_norm, y_norm,z_norm);
  }
  
  //If the vector is longer than maxL, shrink it to be maxL otherwise do nothing
  public void clampToLength(float maxL){
    if(this.length() > maxL){
      this.setToLength(maxL);
    }
  }
  
  //Grow or shrink the vector have a length of maxL
  public void setToLength(float newL){
    float curL = this.length();
    this.x *= newL / curL;
    this.y *= newL / curL;
    this.z *= newL / curL;
  }
  
  public float distanceTo(Vec3 rhs){
    return (float)Math.sqrt(Math.pow(this.x - rhs.x,2.0) + Math.pow(this.y - rhs.y, 2.0) + Math.pow(this.z - rhs.z, 2.0));
  }
}

Vec3 interpolate(Vec3 a, Vec3 b, float t){
  float newX = (a.x + b.x) * t;
  float newY = (a.y + b.y) * t;
  float newZ = (a.z + b.z) * t;
  return new Vec3(newX, newY, newZ); 
}

float dot(Vec3 a, Vec3 b){
  return (a.x*b.x) + (a.y*b.y) + (a.z*b.z);
}

Vec3 cross(Vec3 a, Vec3 b){
  float newX,newY,newZ;
  newX = (a.y*b.z)-(a.z*b.y);
  newY = (a.z*b.x)-(a.x*b.z);
  newZ = (a.x*b.y)-(a.y*b.x);
  return new Vec3(newX,newY,newZ);
}

Vec3 projAB(Vec3 a, Vec3 b){
  float curBL = b.length();
  float newX = (float)(((dot(b,a))/(Math.pow(curBL,2))) * b.x);
  float newY = (float)(((dot(b,a))/(Math.pow(curBL,2))) * b.y);
  float newZ = (float)(((dot(b,a))/(Math.pow(curBL,2))) * b.z);
  return new Vec3(newX, newY,newZ);
}
