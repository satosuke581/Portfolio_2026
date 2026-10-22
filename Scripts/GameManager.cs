using UnityEngine;
using UnityEngine.InputSystem;
using System.Collections;
using System.Collections.Generic;
using System;
using UnityEngine.SceneManagement;
using TMPro;

public class GameManager : MonoBehaviour
{
    public GameObject OthelloCell, BlackStone, WhiteStone, YellowStar, RedStar, result;
    public TextMeshProUGUI resultText;
    static int ReverseNum = 4;
    public bool isHard = false;
    int MyReverseNum, ComReverseNum, StoneNum = 4;
    GameObject[] MyStars = new GameObject[ReverseNum];
    GameObject[] ComStars = new GameObject[ReverseNum];
    GameObject BlackStoneNow, WhiteStoneNow;
    OthelloCellCOntroller[,] cnt = new OthelloCellCOntroller[8,8];
    int turn = 0;//0:黒のターン、1:白のターン
    int Myturn = 0;
    // Start is called once before the first execution of Update after the MonoBehaviour is created
    void Start()
    {
        MyReverseNum = ReverseNum;
        ComReverseNum = ReverseNum;
        isHard = false;
        //オセロボードを作る
        for(int i = 0; i < 8; i++)
        {
            for(int j = 0; j < 8; j++)
            {
                cnt[i,j] = Instantiate(OthelloCell, new Vector2(i, j), Quaternion.identity).GetComponent<OthelloCellCOntroller>();
            }
        }
        //最初の4つの石を置く
        cnt[3,3].PutStone(0);
        cnt[4,4].PutStone(0);
        cnt[3,4].PutStone(1);
        cnt[4,3].PutStone(1);
        //ターンを示す黒石を置く　
        BlackStoneNow = Instantiate(BlackStone, new Vector3(8.5f,7,0), Quaternion.identity);
        //残り裏返し回数を示す星を置く
        for(int i = 0; i < ReverseNum; i++)
        {
            MyStars[i] = Instantiate(YellowStar, new Vector2(12.5f+i, 5), Quaternion.identity);
            ComStars[i] = Instantiate(RedStar, new Vector2(12.5f+i, 4), Quaternion.identity);
        }
    }

    // Update is called once per frame
    void Update()
    {
        if(turn == Myturn){
            if (Keyboard.current.hKey.wasPressedThisFrame) isHard = true;
            PutMyStone();
        }
    }
    void PutMyStone()
    {
        if (Mouse.current.leftButton.wasPressedThisFrame)//もし左クリックされたら
        {
            //クリックした座標を取得
            Vector2 mousePos = Mouse.current.position.ReadValue();
            Vector3 worldPos = Camera.main.ScreenToWorldPoint(mousePos);
            worldPos.z = 0;
            //座標をオセロボードの左下を(0,0)とするものに変換
            worldPos.x += 0.5f;
            worldPos.y += 0.5f;
            if(worldPos.x < 0 || worldPos.x >=8 || worldPos.y <0 || worldPos.y >=8) return;//座標がオセロボードの外ならリターン
            int x = (int)worldPos.x, y = (int)worldPos.y;
            if (PutStone(x, y)) changeTurn();
        }
    }
    bool PutStone(int x, int y)//石をひっくり返すか置く
    {
        if(cnt[x,y].StoneColor==turn) return false;//すでに自分の石があるならリターン
        if(turn == Myturn && MyReverseNum == 0 && cnt[x,y].StoneColor == 1-turn) return false;//これ以上裏返せない時はリターン
        if(turn == 1-Myturn && ComReverseNum == 0 && cnt[x,y].StoneColor == 1-turn) return false;//同上
        List<(int, int)> reversableStones = SearchReversableStones(x,y);
        foreach(var p in reversableStones) cnt[p.Item1, p.Item2].Reverse();
        //もし石をひっくり返したなら石を置くかひっくり返す
        if (reversableStones.Count > 0)
        {
            //石をおく
            if(cnt[x,y].StoneColor == -1) {cnt[x,y].PutStone(turn);StoneNum++;}
            else if(cnt[x,y].StoneColor == 1-turn) {
                cnt[x,y].Reverse();
                if(turn == Myturn)
                {
                    MyReverseNum--;
                    Destroy(MyStars[MyReverseNum]);
                }
                else if(turn == 1 - Myturn)
                {
                    ComReverseNum--;
                    Destroy(ComStars[ComReverseNum]);
                }
            }
            return true;
        }
        return false;
    }
    List<(int,int)> SearchReversableStones(int x, int y)
    {
        List<(int, int)> ret = new List<(int, int)>();
        for(int dx = -1; dx < 2; dx++)
            {
                for(int dy = -1; dy < 2; dy++)
                {
                    int x_ = x + dx, y_ = y + dy;
                    if(dx == dy && dx == 0)continue;
                    //(dx,dy)の方向でひっくり返せる石があるか確認する
                    int last_x = x, last_y = y;
                    while(x_>=0 && x_ <=7 && y_>=0 && y_ <= 7)
                    {
                        if (cnt[x_, y_].StoneColor == -1) break;
                        if (cnt[x_, y_].StoneColor == turn)
                        {
                            last_x = x_;last_y=y_;
                            break;
                        }
                        x_ += dx;
                        y_ += dy;
                    }
                    while(last_x != x || last_y != y)
                    {
                        if(cnt[last_x, last_y].StoneColor == 1-turn) {
                            ret.Add((last_x, last_y));
                        }
                        last_x -= dx;
                        last_y -= dy;
                    }
                }
            }
        return ret;
    }
    int[,] CellValue()//各マスの評価値(ひっくり返せる石の数、石を変更できないなら0)を算出する(ret[8,8]には打てる手の数を入れる)
    {
        int[,] ret = new int[9,9];
        ret[8,8]=0;
        for(int i = 0;i<8;i++) for(int j = 0; j < 8; j++)
        {
            ret[i,j] = SearchReversableStones(i,j).Count;
            if(cnt[i,j].StoneColor==turn) ret[i,j]=0;//そのターンの人の石ならvalue=0
            if(turn == Myturn && MyReverseNum == 0 && cnt[i,j].StoneColor == 1-turn) ret[i,j]=0;//裏返せないならvalue=0
            if(turn == 1-Myturn && ComReverseNum == 0 && cnt[i,j].StoneColor == 1-turn) ret[i,j]=0;//同上
            if(ret[i,j]>0) ret[8,8]++;
        }
        return ret;
    }
    IEnumerator PutComStone()
    {
        yield return new WaitForSeconds(1f); // 1秒待つ
        if (StoneNum >= 56 || !isHard)//終盤ならいっぱいひっくり返せるように動く
        {
            var val = CellValue();
            int x_mx = 0, y_mx = 0;
            for(int i = 0;i<8;i++) for(int j = 0; j < 8; j++)
            {
                if (val[i, j] > val[x_mx, y_mx])
                {
                    x_mx=i;
                    y_mx=j;
                }   
            }
            PutStone(x_mx,y_mx);
            changeTurn();
        }
        else//序盤は相手の手を奪う
        {
            int disMx = -1000, x_mx=-1, y_mx=-1;
            var cellValue = CellValue();
            for(int i = 0;i<8;i++) for(int j = 0; j < 8; j++)
            {
                int cntij = cnt[i,j].StoneColor;
                if(cellValue[i,j]==0) continue;
                List<(int, int)> reversableStones = SearchReversableStones(i,j);
                foreach(var p in reversableStones) cnt[p.Item1, p.Item2].StoneColor = 1-cnt[p.Item1, p.Item2].StoneColor;
                cnt[i,j].StoneColor=turn;
                int befCnt = CellValue()[8,8];
                turn = 1-turn;//擬似的にターンも変える
                int afCnt = CellValue()[8,8];
                //変えたものもどす
                foreach(var p in reversableStones) cnt[p.Item1, p.Item2].StoneColor = 1-cnt[p.Item1, p.Item2].StoneColor;
                cnt[i,j].StoneColor = cntij;
                turn = 1-turn;
                if(befCnt - afCnt > disMx)
                {
                    x_mx = i;
                    y_mx = j;
                }
            }
            PutStone(x_mx,y_mx);
            changeTurn();
        }
    }
    bool changeTurn()
    {
        turn = 1 - turn;
        if (CellValue()[8, 8] == 0)//どこもひっくり返せないなら
        {
            turn = 1-turn;//ターンを交代前に戻す
            if(CellValue()[8, 8] == 0)//それでもどこもひっくり返せないなら決着！！！！！
            {
                GameObject[] objectsToHide = GameObject.FindGameObjectsWithTag("UI");
                foreach(var obj in objectsToHide) obj.SetActive(false); 
                if(turn == 0) Destroy(BlackStoneNow);
                else if(turn == 1) Destroy(WhiteStoneNow);
                result.SetActive(true);
                //白マスと黒マスをそれぞれ集計して結果を表示する
                int w = 0, b = 0;
                for(int i = 0;i<8;i++) for(int j = 0; j < 8; j++)
                {
                    if(cnt[i,j].StoneColor==0) b++;
                    else if(cnt[i,j].StoneColor==1) w++;
                }
                if(b==w) resultText.text = "あなた:" + b + "個\n相手:" + w + "個\n" + "引き分け";
                else if(b>w)resultText.text = "あなた:" + b + "個\n相手:" + w + "個\n" + "勝ち";
                else resultText.text = "あなた:" + b + "個\n相手:" + w + "個\n" + "負け";
                return false;
            }
            if(turn != Myturn) StartCoroutine(PutComStone());
            return false;
        }
        if(turn != Myturn) StartCoroutine(PutComStone());
        if (turn == 1)
        {
            Destroy(BlackStoneNow);
            WhiteStoneNow = Instantiate(WhiteStone, new Vector3(8.5f,7,0), Quaternion.identity);
        }
        else if(turn == 0)
        {
            Destroy(WhiteStoneNow);
            BlackStoneNow = Instantiate(BlackStone, new Vector3(8.5f,7,0), Quaternion.identity);
        }
        return true;
    }
    public void ResetScene()//リセットする
    {
        SceneManager.LoadScene(SceneManager.GetActiveScene().name);
    }
}
