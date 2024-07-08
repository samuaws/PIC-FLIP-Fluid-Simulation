using UnityEngine;
using TMPro;

public class CellVisualizer : MonoBehaviour
{
    public TextMeshProUGUI cellInfoText; // Reference to a UI Text component to display cell information

    public void DisplayCellInfo(Vector2 position, Vector2 velocity, bool hasFluid)
    {
        string info = $"Position: {position}\nVelocity: {velocity}\nHas Fluid: {hasFluid}";
        cellInfoText.text = info;
    }
}
