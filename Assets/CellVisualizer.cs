using UnityEngine;
using TMPro;

public class CellVisualizer : MonoBehaviour
{
    public TextMeshProUGUI cellInfoText; // Reference to a UI Text component to display cell information

    public void DisplayCellInfo(Vector2 position, Vector2 velocity, bool hasFluid, float distanceToBoundary, bool isBoundary)
    {
        string fluidInfo = hasFluid ? "Has Fluid" : "No Fluid";
        string boundaryInfo = isBoundary ? "Boundary Cell" : "Non-Boundary Cell";
        string info = $"Position: {position}\nVelocity: {velocity}\n{fluidInfo}\nDistance to Boundary: {distanceToBoundary}\n{boundaryInfo}";
        cellInfoText.text = info;
    }
}
