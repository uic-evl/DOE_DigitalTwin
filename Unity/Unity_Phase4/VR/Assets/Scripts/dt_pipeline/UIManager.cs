using UnityEngine;
using UnityEngine.UI;

public class UIManager : MonoBehaviour
{
    [SerializeField] private Button startConsumerButton;
    [SerializeField] private Button stopConsumerButton;

    [SerializeField] private Button startProducerButton;
    [SerializeField] private Button stopProducerButton;

    private void Start()
    {
        EventManager.myEvents.onStartConsumer.AddListener(() => stopConsumerButton.interactable = true);
        EventManager.myEvents.onStopConsumer.AddListener(() => startConsumerButton.interactable = true);

        EventManager.myEvents.onStartProducer.AddListener(() => stopProducerButton.interactable = true);
        EventManager.myEvents.onStopProducer.AddListener(() => startProducerButton.interactable = true);
    
        stopConsumerButton.interactable = false;
        stopProducerButton.interactable = false;

        //Add onClick() events
        startConsumerButton.onClick.AddListener(() =>
        {
            startConsumerButton.interactable = false;
            EventManager.myEvents.onStartConsumer.Invoke();
        });
    
        stopConsumerButton.onClick.AddListener(() =>
        {
            stopConsumerButton.interactable = false;
            EventManager.myEvents.onStopConsumer.Invoke();
        });

        startProducerButton.onClick.AddListener(() =>
        {
            Debug.Log("Producer pressed");
            startProducerButton.interactable = false;
            EventManager.myEvents.onStartProducer.Invoke();
        });
    
        stopProducerButton.onClick.AddListener(() =>
        {
            stopProducerButton.interactable = false;
            EventManager.myEvents.onStopProducer.Invoke();
        });
    }
}

