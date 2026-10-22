using UnityEngine;

public class ExplanationManager : MonoBehaviour
{
    public GameObject explanationPanel;
    // Start is called once before the first execution of Update after the MonoBehaviour is created
    void Start()
    {
        
    }

    // Update is called once per frame
    void Update()
    {
        
    }
    public void ShowExplanation()
    {
        explanationPanel.SetActive(true);
    }

    public void HideExplanation()
    {
        explanationPanel.SetActive(false);
    }
}
