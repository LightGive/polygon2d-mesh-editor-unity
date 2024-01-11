using System.Linq;
using UnityEngine;
using UnityEngine.InputSystem;
using System.Collections.Generic;


#if UNITY_EDITOR
using UnityEditor;
#endif

namespace UnityEditor
{
    [ExecuteInEditMode]
    [RequireComponent(typeof(MeshRenderer))]
    [RequireComponent(typeof(MeshFilter))]
    public class Polygon2D : MonoBehaviour
    {
        [SerializeField]
        Vector2[] _points = new Vector2[3]
        {
            new Vector2(0.0f,1.0f),
            new Vector2(1f,-0.5f),
            new Vector2(-1f,-0.5f)
        };
        MeshRenderer _meshRenderer = null;
        MeshFilter _meshFilter = null;

#if UNITY_EDITOR

#endif

        void Start()
        {
        }


        void Update()
        {
            if (!Application.isPlaying)
            {
                UpdateMesh();
            }
        }

        public void UpdateMesh()
        {
            _meshRenderer = _meshRenderer ?? gameObject.GetComponent<MeshRenderer>();
            _meshFilter = _meshFilter ?? gameObject.GetComponent<MeshFilter>();
            Mesh mesh = new Mesh();
            mesh.vertices = _points.Select(x => new Vector3(x.x, x.y, 0.0f)).ToArray();
            mesh.triangles = new Triangulator(_points).Triangulate();
            mesh.RecalculateNormals();
            mesh.RecalculateBounds();
            _meshFilter.mesh = mesh;
            Bounds bounds = mesh.bounds;
            mesh.uv = _points.Select(v => new Vector2(v.x / bounds.size.x, v.y / bounds.size.y)).ToArray();
        }
    }

    class Triangulator
    {
        private List<Vector2> mPoints = new List<Vector2>();

        public Triangulator(Vector2[] points)
        {
            mPoints = new List<Vector2>(points);
        }

        public int[] Triangulate()
        {
            List<int> indices = new List<int>();

            int n = mPoints.Count;
            if (n < 3) return indices.ToArray();

            int[] V = new int[n];
            if (Area() > 0)
            {
                for (int v = 0; v < n; v++)
                    V[v] = v;
            }
            else
            {
                for (int v = 0; v < n; v++)
                    V[v] = (n - 1) - v;
            }

            int nv = n;
            int count = 2 * nv;
            for (int m = 0, v = nv - 1; nv > 2;)
            {
                if ((count--) <= 0)
                    return indices.ToArray();

                int u = v;
                if (nv <= u)
                    u = 0;
                v = u + 1;
                if (nv <= v)
                    v = 0;
                int w = v + 1;
                if (nv <= w)
                    w = 0;

                if (Snip(u, v, w, nv, V))
                {
                    int a, b, c, s, t;
                    a = V[u];
                    b = V[v];
                    c = V[w];
                    indices.Add(a);
                    indices.Add(b);
                    indices.Add(c);
                    m++;
                    for (s = v, t = v + 1; t < nv; s++, t++)
                        V[s] = V[t];
                    nv--;
                    count = 2 * nv;
                }
            }

            indices.Reverse();
            return indices.ToArray();
        }

        private float Area()
        {
            int n = mPoints.Count;
            float A = 0.0f;
            for (int p = n - 1, q = 0; q < n; p = q++)
            {
                Vector2 pval = mPoints[p];
                Vector2 qval = mPoints[q];
                A += pval.x * qval.y - qval.x * pval.y;
            }
            return (A * 0.5f);
        }

        private bool Snip(int u, int v, int w, int n, int[] V)
        {
            int p;
            Vector2 A = mPoints[V[u]];
            Vector2 B = mPoints[V[v]];
            Vector2 C = mPoints[V[w]];
            if (Mathf.Epsilon > (((B.x - A.x) * (C.y - A.y)) - ((B.y - A.y) * (C.x - A.x))))
                return false;
            for (p = 0; p < n; p++)
            {
                if ((p == u) || (p == v) || (p == w))
                    continue;
                Vector2 P = mPoints[V[p]];
                if (InsideTriangle(A, B, C, P))
                    return false;
            }
            return true;
        }

        private bool InsideTriangle(Vector2 A, Vector2 B, Vector2 C, Vector2 P)
        {
            float ax, ay, bx, by, cx, cy, apx, apy, bpx, bpy, cpx, cpy;
            float cCROSSap, bCROSScp, aCROSSbp;

            ax = C.x - B.x; ay = C.y - B.y;
            bx = A.x - C.x; by = A.y - C.y;
            cx = B.x - A.x; cy = B.y - A.y;
            apx = P.x - A.x; apy = P.y - A.y;
            bpx = P.x - B.x; bpy = P.y - B.y;
            cpx = P.x - C.x; cpy = P.y - C.y;

            aCROSSbp = ax * bpy - ay * bpx;
            cCROSSap = cx * apy - cy * apx;
            bCROSScp = bx * cpy - by * cpx;

            return ((aCROSSbp >= 0.0f) && (bCROSScp >= 0.0f) && (cCROSSap >= 0.0f));
        }
    }

#if UNITY_EDITOR
    [CustomEditor(typeof(Polygon2D))]
    public class Polygon2DEditor : Editor
    {
        const float HandleSize = 0.05f;
        readonly Color DotColor = Color.cyan;
        readonly Color PolygonColor = new Color(0.0f, 1.0f, 1.0f, 0.2f);
        Vector3 handlePosition;
        SerializedProperty _pointsProp = null;
        void OnEnable()
        {
            _pointsProp = serializedObject.FindProperty("_points");
        }

        void OnSceneGUI()
        {
            if (_pointsProp.arraySize < 3) { return; }

            Handles.color = DotColor;
            var points = new Vector3[_pointsProp.arraySize];
            var isUpdateVertex = false;
            var t = ((MonoBehaviour)target).transform;

            for (var i = 0; i < _pointsProp.arraySize; i++)
            {
                // 2Dのスライドするハンドルを表示
                EditorGUI.BeginChangeCheck();
                var prop = _pointsProp.GetArrayElementAtIndex(i);
                var pos = (Vector3)prop.vector2Value;
                Vector3 value = Handles.Slider2D(
                    t.TransformPoint(pos),
                    Vector3.forward,
                    Vector3.right,
                    Vector3.up,
                    HandleUtility.GetHandleSize(t.TransformPoint(pos)) * HandleSize,
                    Handles.DotHandleCap, 0);
                points[i] = HandleUtility.WorldToGUIPoint(value);

                Handles.Label(t.TransformPoint(pos) + new Vector3(0f, 0.3f, 0f), $"{i}", GUI.skin.label);
                if (EditorGUI.EndChangeCheck() && !float.IsNaN(value.x) && !float.IsNaN(value.y) && !float.IsNaN(value.z))
                {
                    isUpdateVertex = true;
                    prop.vector2Value = (Vector2)t.InverseTransformPoint(value);
                    serializedObject.ApplyModifiedProperties();
                }
            }

            var mousePos = Event.current.mousePosition;
            (int index, float distance, Vector2 pos) near = (0, float.MaxValue, Vector2.zero);
            for (var i = 0; i < points.Length; i++)
            {
                var nextIdx = (i + 1) % points.Length;
                var nearPoint = GetNearPointDistance(mousePos, points[i], points[nextIdx]);
                var dis = Vector2.Distance(mousePos, nearPoint);
                if (dis > near.distance)
                {
                    continue;
                }
                near = (i, dis, nearPoint);
            }

            Vector3 a = Handles.Slider2D(
                GUIPointToWorldPos(near.pos),
                Vector3.forward,
                Vector3.right,
                Vector3.up,
                HandleUtility.GetHandleSize(GUIPointToWorldPos(near.pos)) * HandleSize,
                Handles.DotHandleCap, 0);



            if (isUpdateVertex)
            {
                ((Polygon2D)target).UpdateMesh();
            }
        }

        Vector3 GUIPointToWorldPos(Vector2 pos)
        {
            var ray = HandleUtility.GUIPointToWorldRay(pos);
            var x0 = ray.origin;
            var m = ray.direction;
            var t = ((Polygon2D)target).transform;
            var n = -t.forward;
            var x = t.position;
            var h = Vector3.Dot(n, x);
            return x0 + ((h - Vector3.Dot(x, x0)) / Vector3.Dot(n, m)) * m;
        }



        Vector2 GetNearPointDistance(Vector2 checkPoint, Vector2 p1, Vector2 p2)
        {
            var v = (p2 - p1).normalized;
            var d = Vector2.Dot(v, checkPoint - p1);
            return p1 + v * Mathf.Clamp(d, 0, Vector2.Distance(p1, p2));
        }
    }
#endif
}
