using System.Linq;
using UnityEngine;
using UnityEngine.InputSystem;

#if UNITY_EDITOR
using UnityEditor;
#endif

namespace UnityEditor
{
    public class Polygon2D : MonoBehaviour
    {
        [SerializeField] Vector2[] _points = new Vector2[3];

        void Start()
        {
        }
    }

#if UNITY_EDITOR
    [CustomEditor(typeof(Polygon2D))]
    public class Polygon2DEditor : Editor
    {
        readonly Color DotColor = Color.cyan;
        readonly Color PolygonColor = new Color(0.0f, 1.0f, 1.0f, 0.2f);
        SerializedProperty _pointsProp = null;
        void OnEnable()
        {
            _pointsProp = serializedObject.FindProperty("_points");
        }

        void OnSceneGUI()
        {
            Handles.color = DotColor;
            var points = new Vector3[_pointsProp.arraySize];
            for (var i = 0; i < _pointsProp.arraySize; i++)
            {
                // 2Dのスライドするハンドルを表示
                EditorGUI.BeginChangeCheck();
                var prop = _pointsProp.GetArrayElementAtIndex(i);
                var pos = (Vector3)prop.vector2Value;
                points[i] = pos;
                Vector3 value = Handles.Slider2D(pos, Vector3.back, Vector3.right, Vector3.up, HandleUtility.GetHandleSize(pos) * 0.05f, Handles.DotHandleCap, 0);
                if (EditorGUI.EndChangeCheck())
                {
                    prop.vector2Value = (Vector2)value;
                    serializedObject.ApplyModifiedProperties();
                }
            }

            Handles.color = PolygonColor;
            Handles.DrawAAConvexPolygon(points);

            var sceneCamera = SceneView.currentDrawingSceneView.camera;
            Ray worldRay = HandleUtility.GUIPointToWorldRay(Mouse.current.position.ReadValue());
            var points2D = points.Select(x => (Vector2)sceneCamera.WorldToScreenPoint(x)).ToArray();
        }
    }
#endif
}