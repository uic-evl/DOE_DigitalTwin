using UnityEngine;
using UnityEngine.UI;

public class UIManagerIK : MonoBehaviour
{
    [SerializeField] private Button startProducerButton;
    [SerializeField] private Button stopProducerButton;

    public EventManagerIK EventManagerIK;

    private void Start()
    {
        EventManagerIK.onStartProducer.AddListener(() => stopProducerButton.interactable = true);
        EventManagerIK.onStopProducer.AddListener(() => startProducerButton.interactable = true);
        stopProducerButton.interactable = false;

        //Add onClick() events
        startProducerButton.onClick.AddListener(() =>
        {
            Debug.Log("Producer pressed");
            startProducerButton.interactable = false;
            EventManagerIK.onStartProducer.Invoke();
        });
    
        stopProducerButton.onClick.AddListener(() =>
        {
            stopProducerButton.interactable = false;
            EventManagerIK.onStopProducer.Invoke();
        });
    }
}

