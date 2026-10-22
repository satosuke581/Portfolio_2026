using UnityEngine;

public class CloudController : MonoBehaviour
{
    public float moveSpeed;
    // Start is called once before the first execution of Update after the MonoBehaviour is created
    void Start()
    {
        
    }

    // Update is called once per frame
    void Update()
    {
        Vector3 pos = this.transform.position;
        pos.x -= moveSpeed * Time.deltaTime;
        this.transform.position = pos;
        if(this.transform.position.x < -10)
        {
            pos = this.transform.position;
            pos.x = 25;
            this.transform.position = pos;
        }
    }
}
