using UnityEngine;

public class OthelloCellCOntroller : MonoBehaviour
{
    public GameObject BlackStone, WhiteStone;
    GameObject BlackStoneNow, WhiteStoneNow;
    public int StoneColor = -1;//-1:石なし、0:黒、1:白
    // Start is called once before the first execution of Update after the MonoBehaviour is created
    void Start()
    {
        
    }

    // Update is called once per frame
    void Update()
    {
        
    }
    public void PutStone(int color)
    {
        if(color == 0) BlackStoneNow = Instantiate(BlackStone, this.transform.position, Quaternion.identity);
        else if(color == 1) WhiteStoneNow = Instantiate(WhiteStone, this.transform.position, Quaternion.identity);
        StoneColor = color;
    }
    public void Reverse()
    {
        if(StoneColor == -1) return;
        else if(StoneColor == 0) {
            Destroy(BlackStoneNow);
            PutStone(1);
        }
        else if(StoneColor == 1) {
            Destroy(WhiteStoneNow);
            PutStone(0);
        }
    }
}
