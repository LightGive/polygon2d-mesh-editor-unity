using System.Linq;
using UnityEngine;
using UnityEngine.InputSystem;
using System.Collections.Generic;
using UnityEngine.Assertions;



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
        static readonly Vector2[] DefaultPoints = new Vector2[]
        {
            new Vector2(0.0f,1.0f),
            new Vector2(1f,-0.5f),
            new Vector2(-1f,-0.5f)
        };

        [SerializeField] Vector2[] _points = DefaultPoints;

        MeshRenderer _meshRenderer = null;
        MeshFilter _meshFilter = null;

        void Start()
        {
            UpdateMesh();
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
        private List<Vector2> _pointList = new List<Vector2>();

        public Triangulator(Vector2[] points)
        {
            _pointList = new List<Vector2>(points);
        }

        public int[] Triangulate()
        {
            List<int> indices = new List<int>();

            int n = _pointList.Count;
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
            int n = _pointList.Count;
            float a = 0.0f;
            for (int p = n - 1, q = 0; q < n; p = q++)
            {
                Vector2 pval = _pointList[p];
                Vector2 qval = _pointList[q];
                a += pval.x * qval.y - qval.x * pval.y;
            }
            return a * 0.5f;
        }

        private bool Snip(int u, int v, int w, int n, int[] V)
        {
            int p;
            Vector2 a = _pointList[V[u]];
            Vector2 b = _pointList[V[v]];
            Vector2 c = _pointList[V[w]];
            if (Mathf.Epsilon > (((b.x - a.x) * (c.y - a.y)) - ((b.y - a.y) * (c.x - a.x))))
                return false;
            for (p = 0; p < n; p++)
            {
                if ((p == u) || (p == v) || (p == w))
                {
                    continue;
                }
                Vector2 P = _pointList[V[p]];
                if (InsideTriangle(a, b, c, P))
                {
                    return false;
                }
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
        /// <summary>
        /// ハンドルの大きさ
        /// </summary>
        const float HandleScale = 0.05f;
        /// <summary>
        /// メッシュ編集中、辺との距離がこの値より大きい時ハンドルを非表示にする
        /// </summary>
        const float HandleHideSideDistance = 80.0f;
        /// <summary>
        /// メッシュ編集中、頂点とマウスの位置がこの値より近い時ハンドルを非表示にする
        /// 0~HandleHideSideDistanceの範囲
        /// </summary>
        const float HandleHidePointDistance = 20.0f;
        readonly Color HandleColorVertex = new Color(0f, 1f, 1f);
        readonly Color HandleColorEditing = new Color(0f, 1f, 1f);
        readonly Color HandleColorDelete = new Color(1f, 0f, 0f);
        SerializedProperty _pointsProp = null;

        bool _cached = false;
        bool _isEditing = false;
        int _addVertexControlId = 0;
        int[] _controlIds = new int[0];
        KeyCode _deleteKeycode = KeyCode.None;
        Transform _targetTransform = null;
        float GetHandleSize(Vector3 pos) => HandleUtility.GetHandleSize(pos) * HandleScale;

        void OnEnable()
        {
            Assert.IsTrue(HandleHidePointDistance <= HandleHideSideDistance);
            _pointsProp = serializedObject.FindProperty("_points");
            _isEditing = false;
        }

        void OnSceneGUI()
        {
            if (_pointsProp.arraySize >= 3 || !_isEditing)
            {
                return;
            }

            if (!_cached)
            {
                // ControlIdを割り振る
                _cached = true;
                _deleteKeycode = KeyCode.None;
                _targetTransform = ((Polygon2D)target).transform;
                SetControlId();
            }

            if (_controlIds.Length != _pointsProp.arraySize)
            {
                SetControlId();
            }

            var e = Event.current;
            CheckDeleteFlag(e);
            var points = new Vector3[_pointsProp.arraySize];
            for (var i = 0; i < _pointsProp.arraySize; i++)
            {
                var prop = _pointsProp.GetArrayElementAtIndex(i);
                points[i] = _targetTransform.TransformPoint((Vector3)prop.vector2Value);
            }

            var mousePos = e.mousePosition;

            // 一番近い辺を見つける
            (int index, float distance, Vector2 pos) near = (0, float.MaxValue, Vector2.zero);
            for (var i = 0; i < points.Length; i++)
            {
                var nextIdx = (i + 1) % points.Length;
                var nearPoint = GetNearPointDistance(
                    mousePos,
                    HandleUtility.WorldToGUIPoint(points[i]),
                    HandleUtility.WorldToGUIPoint(points[nextIdx]));
                var dis = Vector2.Distance(mousePos, nearPoint);
                if (dis > near.distance)
                {
                    continue;
                }
                near = (i, dis, nearPoint);
            }

            var p1 = Vector2.Distance(near.pos, HandleUtility.WorldToGUIPoint(points[near.index]));
            var p2 = Vector2.Distance(near.pos, HandleUtility.WorldToGUIPoint(points[(near.index + 1) % points.Length]));

            // 辺とマウスの位置が近いかどうか
            var isNearSide = near.distance < HandleHideSideDistance;
            var nearIdx = p1 < p2 ?
                near.index :
                (near.index + 1) % points.Length;

            for (var i = 0; i < points.Length; i++)
            {
                var id = _controlIds[i];

                // 頂点削除チェック
                if (e.type == EventType.MouseDown && HandleUtility.nearestControl == id && _deleteKeycode != KeyCode.None)
                {
                    DeleteVertex(i);
                    return;
                }

                Handles.color =
                    _deleteKeycode != KeyCode.None && isNearSide && i == nearIdx ?
                    HandleColorDelete :
                    HandleColorVertex;

                // 2Dのスライドするハンドルを表示
                EditorGUI.BeginChangeCheck();
                var updatePos = Handles.Slider2D(
                    id,
                    points[i],
                    _targetTransform.forward,
                    _targetTransform.right,
                    _targetTransform.up,
                    GetHandleSize(points[i]),
                    Handles.DotHandleCap,
                    Vector2.zero);

                if (EditorGUI.EndChangeCheck() && !float.IsNaN(updatePos.x) && !float.IsNaN(updatePos.y) && !float.IsNaN(updatePos.z))
                {
                    var prop = _pointsProp.GetArrayElementAtIndex(i);
                    prop.vector2Value = (Vector2)_targetTransform.InverseTransformPoint(updatePos);
                    serializedObject.ApplyModifiedProperties();
                }
            }

            // 頂点と近すぎる時や、辺との距離が遠すぎる時は表示しない
            if (near.distance < HandleHideSideDistance &&
                p1 > HandleHidePointDistance &&
                p2 > HandleHidePointDistance &&
                _deleteKeycode == KeyCode.None)
            {
                var nearWorldPos = GUIPointToWorldPos(near.pos);
                if (e.type == EventType.MouseDown && HandleUtility.nearestControl == _addVertexControlId)
                {
                    InsertPosition(near.index, _targetTransform.InverseTransformPoint(nearWorldPos));
                    SetControlId(near.index + 1);
                    return;
                }

                Handles.color = HandleColorEditing;
                var targetPos = GUIPointToWorldPos(near.pos);
                Handles.Slider2D(
                    _addVertexControlId,
                    nearWorldPos,
                    _targetTransform.forward,
                    _targetTransform.right,
                    _targetTransform.up,
                    GetHandleSize(nearWorldPos),
                    Handles.DotHandleCap,
                    Vector2.zero);

                // string controlIdsStr = "";
                // for (var i = 0; i < _controlIds.Length; i++)
                // {
                //     controlIdsStr += $"[{i}]:{_controlIds[i]}\n";
                // }
                // controlIdsStr += $"addVertex:{_addVertexControlId}";
                // Debug.Log(controlIdsStr);
            }

            SceneView.RepaintAll();
        }

        public override void OnInspectorGUI()
        {
            EditorGUI.BeginChangeCheck();
            _isEditing = GUILayout.Toggle(_isEditing, _isEditing ? "編集中" : "メッシュを編集する", "Button");
            if (EditorGUI.EndChangeCheck())
            {
                if (_isEditing)
                {
                    EditVertexStart();
                }
                else
                {
                    EditVertexEnd();
                }
            }

            if (_isEditing)
            {
                serializedObject.Update();
            }
            base.OnInspectorGUI();
        }

        /// <summary>
        /// 頂点削除フラグのON,OFFの切り替えチェック
        /// </summary>
        /// <param name="e"></param>
        void CheckDeleteFlag(Event e)
        {
            if (e.type == EventType.KeyDown &&
                _deleteKeycode == KeyCode.None && (
                e.keyCode == KeyCode.LeftControl ||
                e.keyCode == KeyCode.RightControl ||
                e.keyCode == KeyCode.LeftCommand ||
                e.keyCode == KeyCode.RightCommand))
            {
                _deleteKeycode = e.keyCode;
            }
            else if (e.type == EventType.KeyUp && e.keyCode == _deleteKeycode)
            {
                _deleteKeycode = KeyCode.None;
            }
        }

        void EditVertexStart()
        {
            //Debug.Log("編集開始");
        }

        void EditVertexEnd()
        {
            //Debug.Log("編集終了");
            serializedObject.ApplyModifiedProperties();
        }

        /// <summary>
        /// 頂点を追加
        /// </summary>
        /// <param name="index"></param>
        /// <param name="pos"></param>
        void InsertPosition(int index, Vector2 pos)
        {
            _pointsProp.arraySize++;
            for (var i = _pointsProp.arraySize - 1; i > index; i--)
            {
                var prop = _pointsProp.GetArrayElementAtIndex(i);
                if (i == index + 1)
                {
                    prop.vector2Value = pos;
                }
                else
                {
                    var preProp = _pointsProp.GetArrayElementAtIndex(i - 1);
                    prop.vector2Value = preProp.vector2Value;
                }
            }
            serializedObject.ApplyModifiedProperties();
        }

        /// <summary>
        /// 頂点を削除
        /// </summary>
        /// <param name="index"></param>
        void DeleteVertex(int index)
        {
            if (_pointsProp.arraySize <= 3)
            {
                Debug.Log("最低でもポリゴン数は3つ");
                return;
            }

            for (var i = 0; i < _pointsProp.arraySize; i++)
            {
                if (i <= index)
                {
                    continue;
                }
                _pointsProp.GetArrayElementAtIndex(i - 1).vector2Value =
                _pointsProp.GetArrayElementAtIndex(i).vector2Value;
            }

            _pointsProp.arraySize--;
            serializedObject.ApplyModifiedProperties();
        }

        void SetControlId() => SetControlId(-1);
        void SetControlId(int addVertexIndex)
        {
            _controlIds = new int[_pointsProp.arraySize];
            for (var i = 0; i < _pointsProp.arraySize; i++)
            {
                var id = GUIUtility.GetControlID(i, FocusType.Passive);
                _controlIds[i] = id;
                if (i == addVertexIndex)
                {
                    GUIUtility.hotControl = id;
                }
            }
            _addVertexControlId = GUIUtility.GetControlID(_pointsProp.arraySize, FocusType.Passive);
        }

        Vector3 GUIPointToWorldPos(Vector2 pos)
        {
            var ray = HandleUtility.GUIPointToWorldRay(pos);
            var r0 = ray.origin;
            var m = ray.direction;
            var n = -((Polygon2D)target).transform.forward;
            var h = Vector3.Dot(n, ((Polygon2D)target).transform.position);
            return r0 + (h - Vector3.Dot(n, r0)) / Vector3.Dot(n, m) * m;
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