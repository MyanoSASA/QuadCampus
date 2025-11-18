using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class CourceSelector : MonoBehaviour
{
    [SerializeField] DroneCourseLayoutGenerator courseLayoutGenerator;
    // Start is called before the first frame update
    void Start()
    {
        
    }
    public void SelectRectangular()
    {
        courseLayoutGenerator.ClearLayout();
        courseLayoutGenerator.BuildLayout(DroneCourseMode.Rectangular);
    }
    public void SelectFigureEight()
    {
        courseLayoutGenerator.ClearLayout();
        courseLayoutGenerator.BuildLayout(DroneCourseMode.FigureEight);
    }
    // Update is called once per frame
    void Update()
    {
        
    }
}
