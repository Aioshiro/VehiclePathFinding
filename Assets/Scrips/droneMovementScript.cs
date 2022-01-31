using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class droneMovementScript : MonoBehaviour
{
   public float force;
   public float motionSpeed = 500;
   private float tiltAmount = 0;
   private float tiltVelocity;
   public float sideMotion = 300;
   private float sideTiltAmount;
   private float sideTiltVelocity;
   private float yRotation;
   private float currentY;
   public float rotationAmount = 2.5f;
   private float rotationVelocity;
   private Vector3 SmoothingMotionVelocity;
   Rigidbody droneModel;

    void Awake(){
        droneModel = GetComponent<Rigidbody>();  // when script run we want the drone to get rigidbody component
    }

    void Main(){
        MotionForwardBackward();
        MotionLeftRight();
        MotionUpDown();
        RotationLeftRight();
        SmoothingMotion();

        droneModel.AddRelativeForce(Vector3.up * force);
        droneModel.rotation = Quaternion.Euler(new Vector3(tiltAmount, currentY, sideTiltAmount));
    }

    void MotionForwardBackward(){
        if(Input.GetAxis("Vertical") != 0){
            droneModel.AddRelativeForce(Vector3.forward * Input.GetAxis("Vertical") * motionSpeed);
            tiltAmount = Mathf.SmoothDamp(tiltAmount, 20 * Input.GetAxis("Vertical"), ref tiltVelocity, 0.1f);
        }
    }

    void MotionLeftRight(){
        if(Mathf.Abs(Input.GetAxis("Horizontal")) > 0.2f){
            droneModel.AddRelativeForce(Vector3.right * Input.GetAxis("Horizontal") * sideMotion);
            sideTiltAmount = Mathf.SmoothDamp(sideTiltAmount, -20 * Input.GetAxis("Horizontal"), ref sideTiltVelocity, 0.1f);
        }
        else{
            sideTiltAmount = Mathf.SmoothDamp(sideTiltAmount, 0, ref sideTiltVelocity, 0.1f);  // if not pressing anything the drone should go back to the rotation orignaly
        }
    }

    void MotionUpDown(){
        if(Input.GetKey(KeyCode.N)){
            force = 400;
        }
        else if(Input.GetKey(KeyCode.M)){
            force = -250;
        }
        else if(!Input.GetKey(KeyCode.N) && !Input.GetKey(KeyCode.M)){
            force = 98.2f;  // gravitation*mass
        }
    }

    void RotationLeftRight(){
        if(Input.GetKey(KeyCode.K)){
            yRotation += rotationAmount;
        }
        if(Input.GetKey(KeyCode.J)){
            yRotation -= rotationAmount;
        }

        currentY = Mathf.SmoothDamp(currentY, yRotation, ref rotationVelocity, 0.25f);
    }

    void SmoothingMotion(){
        if(Mathf.Abs(Input.GetAxis("Vertical")) > 0.2f && Mathf.Abs(Input.GetAxis("Horizontal")) > 0.2f){
            droneModel.velocity = Vector3.ClampMagnitude(droneModel.velocity, Mathf.Lerp(droneModel.velocity.magnitude, 10, Time.deltaTime * 2));  // the higher Time.deltaTime the faster it will go from one value to another
        }
        if(Mathf.Abs(Input.GetAxis("Vertical")) < 0.2f && Mathf.Abs(Input.GetAxis("Horizontal")) > 0.2f){
            droneModel.velocity = Vector3.ClampMagnitude(droneModel.velocity, Mathf.Lerp(droneModel.velocity.magnitude, 5, Time.deltaTime * 2));  // slower when moving left or right compared to moving foward and backward
        }
        if(Mathf.Abs(Input.GetAxis("Vertical")) > 0.2f && Mathf.Abs(Input.GetAxis("Horizontal")) < 0.2f){ 
            droneModel.velocity = Vector3.ClampMagnitude(droneModel.velocity, Mathf.Lerp(droneModel.velocity.magnitude, 10, Time.deltaTime * 2));
        }
        if(Mathf.Abs(Input.GetAxis("Vertical")) < 0.2f && Mathf.Abs(Input.GetAxis("Horizontal")) < 0.2f){  // If not pressing any key the drone should stand still
            droneModel.velocity = Vector3.SmoothDamp(droneModel.velocity, Vector3.zero, ref SmoothingMotionVelocity, 0.95f);
        }  
    }
}
