using System.Collections.Generic;
using Unity.VisualScripting;
using UnityEngine;
using static UnityEditor.Experimental.GraphView.Port;
using UnityEngine.XR;

namespace ProceduralModeling_AI
{
    public class ShellSpace : MonoBehaviour
    {
        MeshFilter filt;
        MeshRenderer rend;

        [SerializeField] private Vector3 offsetPos0 = new Vector3(-1.0f, 3.0f, -1.0f);
        [SerializeField] private Vector3 offsetPos1 = new Vector3(1.0f, 3.0f, -1.0f);
        [SerializeField] private Vector3 offsetPos2 = new Vector3(-1.0f, 3.0f, 1.0f);
        [SerializeField] private Vector3 offsetPos3 = new Vector3(1.0f, 3.0f, 1.0f);

        [SerializeField] private Material mat;
        [SerializeField] private Vector3 ray_orig = new Vector3(-2.0f, 3.0f, 0.3f);
        [SerializeField] private Vector3 ray_dir = new Vector3(0.859812f, -0.490839f, -0.140707f);

        private float uLen = 1.0f;
        private float vLen = 1.0f;

        private Vector3 basePos0 = new Vector3(-1.0f, 0.0f, -1.0f);
        private Vector3 basePos1 = new Vector3(1.0f, 0.0f, -1.0f);
        private Vector3 basePos2 = new Vector3(-1.0f, 0.0f, 1.0f);
        private Vector3 basePos3 = new Vector3(1.0f, 0.0f, 1.0f);
        private Vector3 center = new Vector3(5.0f, 0.0f, 0.0f);

        private Vector3 texBasePos0;
        private Vector3 texBasePos1;
        private Vector3 texBasePos2;
        private Vector3 texBasePos3;

        private Vector3 texOffsetPos0;
        private Vector3 texOffsetPos1;
        private Vector3 texOffsetPos2;
        private Vector3 texOffsetPos3;

        private bool inIntersect;
        private bool outIntersect;
        private float tin;
        private float tout;
        private float hin;
        private float hEnd;
        private int MAX_ITERATIONS = 10;
        private int shellOffset = 8;

        // Start is called before the first frame update
        private void Start()
        {
            MeshFilter[] meshFilters = GetComponents<MeshFilter>();
            if (meshFilters.Length == 1)
            {
                // MeshFilterÇ™ë∂ç›Ç∑ÇÈèÍçá
                filt = meshFilters[0];
            }
            else
            {
                // MeshFilterÇ™ë∂ç›ÇµÇ»Ç¢èÍçá
                filt = gameObject.AddComponent<MeshFilter>();
            }

            if (TryGetComponent<MeshRenderer>(out MeshRenderer mr))
            {
                rend = mr;
            }
            else
            {
                rend = this.gameObject.AddComponent<MeshRenderer>();
            }

            texBasePos0 = new Vector3(-uLen, 0.0f, -vLen);
            texBasePos1 = new Vector3(uLen, 0.0f, -vLen);
            texBasePos2 = new Vector3(-uLen, 0.0f, vLen);
            texBasePos3 = new Vector3(uLen, 0.0f, vLen);

            texOffsetPos0 = new Vector3(-uLen, 3.0f, -vLen);
            texOffsetPos1 = new Vector3(uLen, 3.0f, -vLen);
            texOffsetPos2 = new Vector3(-uLen, 3.0f, vLen);
            texOffsetPos3 = new Vector3(uLen, 3.0f, vLen);

            texOffsetPos0 += center;
            texOffsetPos1 += center;
            texOffsetPos2 += center;
            texOffsetPos3 += center;
            texBasePos0 += center;
            texBasePos1 += center;
            texBasePos2 += center;
            texBasePos3 += center;
        }

        private Mesh BuildMesh()
        {
            var mesh = new Mesh();
            var vertices = new List<Vector3>();
            var triangles = new List<int>();

            // shell
            {
                // base
                vertices.Add(basePos0);
                vertices.Add(basePos1);
                vertices.Add(basePos2);
                vertices.Add(basePos3);

                // offset
                vertices.Add(offsetPos0);
                vertices.Add(offsetPos1);
                vertices.Add(offsetPos2);
                vertices.Add(offsetPos3);
            }

            // tex
            {
                // base
                vertices.Add(texBasePos0);
                vertices.Add(texBasePos1);
                vertices.Add(texBasePos2);
                vertices.Add(texBasePos3);

                // offset
                vertices.Add(texOffsetPos0);
                vertices.Add(texOffsetPos1);
                vertices.Add(texOffsetPos2);
                vertices.Add(texOffsetPos3);
            }


            // 2--3
            // |  |
            // |  |
            // 0--1

            // shell
            {
                // base mesh
                triangles.Add(0);
                triangles.Add(1);
                triangles.Add(2);

                //triangles.Add(3);
                //triangles.Add(2);
                //triangles.Add(1);

                // offset mesh
                triangles.Add(4 + 0);
                triangles.Add(4 + 2);
                triangles.Add(4 + 1);

                //triangles.Add(4 + 3);
                //triangles.Add(4 + 1);
                //triangles.Add(4 + 2);

                // side mesh
                // 0 - 1
                triangles.Add(0);
                triangles.Add(4);
                triangles.Add(1);

                triangles.Add(5);
                triangles.Add(1);
                triangles.Add(4);

                // 2 - 0
                triangles.Add(2);
                triangles.Add(6);
                triangles.Add(0);

                triangles.Add(4);
                triangles.Add(0);
                triangles.Add(6);

                // 3 - 2
                //triangles.Add(3);
                //triangles.Add(7);
                //triangles.Add(2);

                //triangles.Add(6);
                //triangles.Add(2);
                //triangles.Add(7);

                // 1 - 3
                //triangles.Add(1);
                //triangles.Add(5);
                //triangles.Add(3);

                //triangles.Add(7);
                //triangles.Add(3);
                //triangles.Add(5);

                // 1 - 2
                triangles.Add(1);
                triangles.Add(5);
                triangles.Add(2);

                triangles.Add(2);
                triangles.Add(5);
                triangles.Add(6);
            }

            // tex
            {
                // base mesh
                triangles.Add(shellOffset + 0);
                triangles.Add(shellOffset + 1);
                triangles.Add(shellOffset + 2);

                //triangles.Add(3);
                //triangles.Add(2);
                //triangles.Add(1);

                // offset mesh
                triangles.Add(shellOffset + 4 + 0);
                triangles.Add(shellOffset + 4 + 2);
                triangles.Add(shellOffset + 4 + 1);

                //triangles.Add(4 + 3);
                //triangles.Add(4 + 1);
                //triangles.Add(4 + 2);

                // side mesh
                // 0 - 1
                triangles.Add(shellOffset + 0);
                triangles.Add(shellOffset + 4);
                triangles.Add(shellOffset + 1);

                triangles.Add(shellOffset + 5);
                triangles.Add(shellOffset + 1);
                triangles.Add(shellOffset + 4);

                // 2 - 0
                triangles.Add(shellOffset + 2);
                triangles.Add(shellOffset + 6);
                triangles.Add(shellOffset + 0);

                triangles.Add(shellOffset + 4);
                triangles.Add(shellOffset + 0);
                triangles.Add(shellOffset + 6);

                // 3 - 2
                //triangles.Add(3);
                //triangles.Add(7);
                //triangles.Add(2);

                //triangles.Add(6);
                //triangles.Add(2);
                //triangles.Add(7);

                // 1 - 3
                //triangles.Add(1);
                //triangles.Add(5);
                //triangles.Add(3);

                //triangles.Add(7);
                //triangles.Add(3);
                //triangles.Add(5);

                // 1 - 2
                triangles.Add(shellOffset + 1);
                triangles.Add(shellOffset + 5);
                triangles.Add(shellOffset + 2);

                triangles.Add(shellOffset + 2);
                triangles.Add(shellOffset + 5);
                triangles.Add(shellOffset + 6);
            }


            mesh.vertices = vertices.ToArray();
            mesh.triangles = triangles.ToArray();
            mesh.RecalculateBounds();

            return mesh;
        }

        float ab_minus_cd(float a, float b, float c, float d)
        {

            float w = d * c;
            float e = -d * c + w;
            float f = a * b - w;
            return f + e;
        }

        bool in_range(float param, float range_min, float range_max)
        {
            return (range_min <= param && param <= range_max);
        }

        bool in_range(float param, float range_min, float range_max, float e)
        {
            return (range_min - e <= param && param <= range_max + e);
        }
        void update_h(float h)
        {
            hin = Mathf.Min(hin, h);
            hEnd = Mathf.Max(hEnd, h);
        }

    private void judgeCoolPatchIntersection(Vector3[] pos)
        {
            // 01 ----------- 11
            // |               |
            // | e00       e11 |
            // |      e10      |
            // 00 ----------- 10
            Vector3 Q00 = pos[0];
            Vector3 q01 = pos[2];
            Vector3 Q10 = pos[1];
            Vector3 q11 = pos[3];
            
            Vector3 e10 = Q10 - Q00;
            Vector3 e11 = q11 - Q10;
            Vector3 e00 = q01 - Q00;

            Vector3 q00 = Q00 - ray_orig;
            Vector3 q10 = Q10 - ray_orig;

            // a + b u + c u^2
            float a = Vector3.Dot(Vector3.Cross(q00, ray_dir), e00);
            float b = Vector3.Dot(Vector3.Cross(q10, ray_dir), e11);
            float c = Vector3.Dot(Vector3.Cross(q01 - q11, ray_dir), e10);

            b -= a + c;
            float det = b * b - 4 * a * c;
            if (0 > det)
            {
                return;
            }

            // Solve for u
            float u1, u2;
            if (c == 0) // trapezoid
            {
                u1 = -a / b;
                u2 = -1;
            }
            else
            {
                float content = Mathf.Sqrt(det) * (b >= 0 ? 1 : -1);
                u1 = (-b - content) / 2;
                u2 = a / u1;
                u1 /= c;
            }

            if (in_range(u1, 0, 1))
            {
                Vector3 pa = (1 - u1) * q00 + u1 * q10; // origin to the intersection point
                Vector3 pb = (1 - u1) * e00 + u1 * e11;
                Vector3 n = Vector3.Cross(ray_dir, pb);
                float n2 = Vector3.Dot(n, n);
                n = Vector3.Cross(n, pa);
                float v1 = Vector3.Dot(n, ray_dir);
                if (0 <= v1 && v1 <= n2)
                {
                    float h = v1 / n2;
                    if (!inIntersect)
                    {
                        tin = Vector3.Dot(n, pb) / n2;
                        hin = h;
                        inIntersect = true;
                    }
                    else
                    {
                        tout = Vector3.Dot(n, pb) / n2;
                        hEnd = h;
                        if (tout < tin)
                        {
                            swap(ref tin, ref tout);
                            swap(ref hin, ref hEnd);
                        }

                        outIntersect = true;
                    }
                }
            }

            if (in_range(u2, 0, 1))
            {
                Vector3 pa = (1 - u2) * q00 + u2 * q10;
                Vector3 pb = (1 - u2) * e00 + u2 * e11;
                Vector3 n = Vector3.Cross(ray_dir, pb);
                float n2 = Vector3.Dot(n, n);
                n = Vector3.Cross(n, pa);
                float v2 = Vector3.Dot(n, ray_dir);
                if (0 <= v2 && v2 <= n2)
                {
                    float h = v2 / n2;
                    if (!inIntersect)
                    {
                        tin = Vector3.Dot(n, pb) / n2;
                        hin = h;
                        inIntersect = true;
                    }
                    else
                    {
                        tout = Vector3.Dot(n, pb) / n2;
                        hEnd = h;
                        if (tout < tin)
                        {
                            swap(ref tin, ref tout);
                            swap(ref hin, ref hEnd);
                        }
                        outIntersect = true;
                    }
                }
            }
        }

        private bool isInsideTriangle(Vector3[] judgedTriangle, Vector3 I, Vector3 n)
        {

            if ((Vector3.Dot(Vector3.Cross(I - judgedTriangle[0], judgedTriangle[1] - judgedTriangle[0]), n) >= 0.0f &&
                 Vector3.Dot(Vector3.Cross(I - judgedTriangle[1], judgedTriangle[2] - judgedTriangle[1]), n) >= 0.0f &&
                 Vector3.Dot(Vector3.Cross(I - judgedTriangle[2], judgedTriangle[0] - judgedTriangle[2]), n) >= 0.0f) ||
                (Vector3.Dot(Vector3.Cross(I - judgedTriangle[0], judgedTriangle[1] - judgedTriangle[0]), n) <= 0.0f &&
                 Vector3.Dot(Vector3.Cross(I - judgedTriangle[1], judgedTriangle[2] - judgedTriangle[1]), n) <= 0.0f &&
                 Vector3.Dot(Vector3.Cross(I - judgedTriangle[2], judgedTriangle[0] - judgedTriangle[2]), n) <= 0.0f))
            {
                return true;
            }

            return false;
        }

        private float evaluate(Vector3 I, float h)
        {
            Vector3[] normals = new Vector3[3];
            normals[0] = offsetPos0 - basePos0;
            normals[1] = offsetPos1 - basePos1;
            normals[2] = offsetPos2 - basePos2;
            Vector3 s = I - basePos0 - h * normals[0];
            Vector3 t = Vector3.Cross(basePos2 + h * normals[2] - basePos0 - h * normals[0],
                basePos1 + h * normals[1] - basePos0 - h * normals[0]);
            float f = Vector3.Dot(s, t);
            return f;
        }

        private float findRoot(Vector3 I, float start, float end)
        {
            Vector3[] normals = new Vector3[3];
            normals[0] = offsetPos0 - basePos0;
            normals[1] = offsetPos1 - basePos1;
            normals[2] = offsetPos2 - basePos2;
            float hr = (start + end) / 2; // init value
            for (int i = 0; i < MAX_ITERATIONS; i++)
            {
                Vector3 s = I - basePos0 - hr * normals[0];
                Vector3 t = Vector3.Cross(basePos1 + hr * normals[1] - basePos0 - hr * normals[0],
                    basePos2 + hr * normals[2] - basePos0 - hr * normals[0]);
                float f = Vector3.Dot(s, t);
                if (Mathf.Abs(f) < 1e-6f)
                {
                    break;
                }

                // update h using Newton's method
                float u = Vector3.Dot(-normals[0], t);
                Vector3 v =
                    Vector3.Cross(basePos2 + hr * normals[2] - basePos0 - hr * normals[0], normals[1] - normals[0]) +
                    Vector3.Cross(basePos1 + hr * normals[1] - basePos0 - hr * normals[0], normals[2] - normals[0]);
                float df = u + Vector3.Dot(s, v);
                float hn = hr - f / df;
                hn = Mathf.Max(start, Mathf.Min(end, hn));
                hr = hn;
            }

            return hr;
        }

        private float solveEquation(Vector3 I)
        {
            Vector3[] normals = new Vector3[3];
            normals[0] = offsetPos0 - basePos0;
            normals[1] = offsetPos1 - basePos1;
            normals[2] = offsetPos2 - basePos2;
            float hStart = -0.2f;
            float hEnd = 1.2f;

            // find the interval
            // https://github.com/pboyer/nomial/blob/main/src/index.ts
            float[] rootsOfDerivative = new float[3];
            int num = 0;
            for (int i = 0; i < 3; i++)
            {
                rootsOfDerivative[i] = 0;
            }

            float a1 = Vector3.Dot(-normals[0], Vector3.Cross(normals[1] - normals[0], normals[2] - normals[0])); // hh
            float b1 = Vector3.Dot(-normals[0],
                Vector3.Cross(normals[1] - normals[0], basePos2 - basePos0) +
                Vector3.Cross(basePos1 - basePos0, normals[2] - normals[0])); // h
            float c1 = Vector3.Dot(-normals[0], Vector3.Cross(basePos1 - basePos0, basePos2 - basePos0));
            float a2 = Vector3.Dot(-normals[0],
                Vector3.Cross(normals[1] - normals[0], normals[2] - normals[0]) +
                Vector3.Cross(normals[1] - normals[0], normals[2] - normals[0])); // hh
            float b2 =
                Vector3.Dot(I - basePos0,
                    Vector3.Cross(normals[1] - normals[0], normals[2] - normals[0]) +
                    Vector3.Cross(normals[1] - normals[0], normals[2] - normals[0])) + Vector3.Dot(-normals[0],
                    Vector3.Cross(normals[1] - normals[0], basePos2 - basePos0) +
                    Vector3.Cross(basePos1 - basePos0, normals[2] - normals[0])); // h
            float c2 = Vector3.Dot(I - basePos0,
                Vector3.Cross(normals[1] - normals[0], basePos2 - basePos0) +
                Vector3.Cross(basePos1 - basePos0, normals[2] - normals[0]));
            float a = a1 + a2;
            float b = b1 + b2;
            float c = c1 + c2;
            float discriminant = b * b - 4 * a * c;
            if (discriminant >= 0)
            {
                float delta = Mathf.Sqrt(discriminant);
                int sign = -1;
                if (b > 0)
                {
                    sign = 1;
                }

                float q = (float)-0.5 * (b + sign * delta);
                float x2 = q / a;
                float x1 = c / q;
                float aa = Mathf.Min(x1, x2);
                float bb = Mathf.Max(x1, x2);
                if (hStart <= aa && aa <= hEnd)
                {
                    rootsOfDerivative[num] = aa;
                    num++;
                }

                if (hStart <= bb && bb <= hEnd)
                {
                    rootsOfDerivative[num] = bb;
                    num++;
                }
            }

            rootsOfDerivative[num] = hEnd;
            num++;

            float intervalStart = hStart;
            float fa = evaluate(I, hStart);
            for (int i = 0; i < num; i++)
            {
                float fb = evaluate(I, rootsOfDerivative[i]);
                float checkSign = fa * fb;
                if (checkSign < 0)
                {
                    // If a solution exists in this range
                    float h = findRoot(I, intervalStart, rootsOfDerivative[i]);
                    return h;
                }

                intervalStart = rootsOfDerivative[i];
                fa = fb;
            }

            return -1.0f;
        }

        private void swap(ref float a, ref float b)
        {
            float tmp = a;
            a = b;
            b = tmp;
        }

        private void judgeTriangleIntersection(Vector3[] judgedTriangle)
        {
            Vector3 n = Vector3.Cross(judgedTriangle[2] - judgedTriangle[0], judgedTriangle[1] - judgedTriangle[0]);
            n.Normalize();
            float t = Vector3.Dot(judgedTriangle[0] - ray_orig, n) / Vector3.Dot(ray_dir, n);
            Vector3 I = ray_orig + t * ray_dir;
            if (isInsideTriangle(judgedTriangle, I, n))
            {
                if (!inIntersect)
                {
                    tin = t;
                    inIntersect = true;
                    hin = solveEquation(I);
                }
                else
                {
                    tout = t;
                    hEnd = solveEquation(I);
                    if (tout < tin)
                    {
                        swap(ref tin, ref tout);
                        swap(ref hin, ref hEnd);
                    }

                    outIntersect = true;
                }
            }
        }

        private void judgePrismIntersetion()
        {
            Vector3[] judgedTriangle = new Vector3[3];
            Vector3[] judgedPatch = new Vector3[4];

            // intersect judge with base triangle
            if (!outIntersect)
            {
                judgedTriangle[0] = basePos0;
                judgedTriangle[1] = basePos1;
                judgedTriangle[2] = basePos2;
                judgeTriangleIntersection(judgedTriangle);
            }

            // intersect judge with offset triangle
            if (!outIntersect)
            {
                judgedTriangle[0] = offsetPos0;
                judgedTriangle[1] = offsetPos1;
                judgedTriangle[2] = offsetPos2;
                judgeTriangleIntersection(judgedTriangle);
            }

            // intersect judge with rectangle p0 p1 p0Offset p1Offset
            if (!outIntersect)
            {
                judgedPatch[0] = basePos0;
                judgedPatch[1] = basePos1;
                judgedPatch[2] = offsetPos0;
                judgedPatch[3] = offsetPos1;
                judgeCoolPatchIntersection(judgedPatch);
            }

            // intersect judge with rectangle p1 p2 p1Offset p2Offset
            if (!outIntersect)
            {
                judgedPatch[0] = basePos1;
                judgedPatch[1] = basePos2;
                judgedPatch[2] = offsetPos1;
                judgedPatch[3] = offsetPos2;
                judgeCoolPatchIntersection(judgedPatch);
            }

            // intersect judge with rectangle p2 p0 p2Offset p0Offset
            if (!outIntersect)
            {
                judgedPatch[0] = basePos2;
                judgedPatch[1] = basePos0;
                judgedPatch[2] = offsetPos2;
                judgedPatch[3] = offsetPos0;
                judgeCoolPatchIntersection(judgedPatch);
            }
        }

        private void transmittanceHDDA(int pattern)
        {
            Vector3[] axes = new Vector3[3];
            axes[0] = new Vector3(1, 0, 0);
            axes[1] = new Vector3(0, 1, 0);
            axes[2] = new Vector3(0, 0, 1);
            ;
            Vector3 ex = (ray_dir.x < ray_dir.y && ray_dir.x < ray_dir.z) ? axes[0] :
                (ray_dir.y < ray_dir.z) ? axes[1] : axes[2];
            Vector3 e1 = Vector3.Cross(ex, ray_dir);
            e1.Normalize();
            Vector3 e0 = Vector3.Cross(ray_dir, e1);
            e0.Normalize();

            Vector3[] normals = new Vector3[3];
            normals[0] = offsetPos0 - basePos0;
            normals[1] = offsetPos1 - basePos1;
            normals[2] = offsetPos2 - basePos2;

            float eE0 = Vector3.Dot(e0, basePos1 - basePos0);
            float eE1 = Vector3.Dot(e0, basePos2 - basePos0);
            float eE2 = Vector3.Dot(e1, basePos1 - basePos0);
            float eE3 = Vector3.Dot(e1, basePos2 - basePos0);

            float eN0 = Vector3.Dot(e0, normals[1] - normals[0]);
            float eN1 = Vector3.Dot(e0, normals[2] - normals[0]);
            float eN2 = Vector3.Dot(e1, normals[1] - normals[0]);
            float eN3 = Vector3.Dot(e1, normals[2] - normals[0]);

            float eo0 = Vector3.Dot(e0, ray_orig - basePos0);
            float eS0 = Vector3.Dot(e0, normals[0]);
            float eo1 = Vector3.Dot(e1, ray_orig - basePos0);
            float eS1 = Vector3.Dot(e1, normals[0]);

            // alpha(h)
            float a2 = eN1 * eS1 - eN3 * eS0;
            float a1 = eN3 * eo0 - eE3 * eS0 + eE1 * eS1 - eN1 * eo1;
            float a0 = eE3 * eo0 - eE1 * eo1;

            // beta(h)
            float b2 = eN2 * eS0 - eN0 * eS1;
            float b1 = eE2 * eS0 - eN2 * eo0 + eN0 * eo1 - eE0 * eS1;
            float b0 = eE0 * eo1 - eE2 * eo0;

            // d(h)
            float d2 = eN0 * eN3 - eN1 * eN2;
            float d1 = eE0 * eN3 + eN0 * eE3 - eE1 * eN2 - eN1 * eE2;
            float d0 = eE0 * eE3 - eE1 * eE2;

            Vector3 na = new Vector3(a2, a1, a0);
            Vector3 nb = new Vector3(b2, b1, b0);
            Vector3 d = new Vector3(d2, d1, d0);
            Vector2 UV0;
            Vector2 E0;
            Vector2 E1;
            Vector2 uvwCenter = new Vector2(center[0] + 0.0f, center[2] + 0.0f);
            //float uLen = 4.0f;
            //float vLen = 4.0f;
            //float2 uvwCenter = make_float2(0.0f, 0.0f);

            // 1 is the largest angle.
            if (pattern == 0)
            {
                UV0 = new Vector2(-uLen, -vLen) + uvwCenter;
                E0 = new Vector2(2 * uLen, 0.0f);
                E1 = new Vector2(0.0f, 2 * vLen);
            }
            else
            {
                UV0 = new Vector2(uLen, -vLen) + uvwCenter;
                E0 = new Vector2(0.0f, 2 * vLen);
                E1 = new Vector2(-2 * uLen, 2 * vLen);
            }

            float u2 = (na * E0.x + nb * E1.x + d * UV0.x).x;
            float u1 = (na * E0.x + nb * E1.x + d * UV0.x).y;
            float u0 = (na * E0.x + nb * E1.x + d * UV0.x).z;
            float v2 = (na * E0.y + nb * E1.y + d * UV0.y).x;
            float v1 = (na * E0.y + nb * E1.y + d * UV0.y).y;
            float v0 = (na * E0.y + nb * E1.y + d * UV0.y).z;

            // print d2, d1, d0, a2, a1, a0
            //Debug.Log("d2:" + d2 + " d1:" + d1 + " d0:" + d0 + " a2:" + a2 + " a1:" + a1 + " a0:" + a0);

            // wlength
            const float scale = 3.0f;
            const float wCenter = 0.0f;
            float delta = 0.01f;
            // DDA
            {
                // swap
                if (hin > hEnd)
                {
                    delta *= -1.0f;
                }

                int count = 0;
                float h = hin;
                float denom;
                Vector3 uvw = new Vector3(0.0f, 0.0f, 0.0f);
                Vector3 uvwPrevious = new Vector3(0.0f, 0.0f, 0.0f);
                {
                    denom = d2 * h * h + d1 * h + d0;
                    uvwPrevious[0] = (u2 * h * h + u1 * h + u0) / denom;
                    uvwPrevious[2] = (v2 * h * h + v1 * h + v0) / denom;
                    uvwPrevious[1] = h * scale - wCenter;
                }

                float tPrevious = tin;
                Color color = new Color(1.0f, 0.0f, 0.0f, 1.0f);
                while (
                    ((hin > hEnd && hEnd < h) || (hin <= hEnd && h < hEnd)) &&
                    count < 3000)
                {
                    h = h + delta;
                    {
                        // u, w, v
                        denom = d2 * h * h + d1 * h + d0;
                        uvw[0] = (u2 * h * h + u1 * h + u0) / denom;
                        uvw[2] = (v2 * h * h + v1 * h + v0) / denom;
                        uvw[1] = h * scale - wCenter;
                    }
                    float alpha = (a2 * h * h + a1 * h + a0) / denom;
                    float beta = (b2 * h * h + b1 * h + b0) / denom;
                    Vector3 s0 = basePos0 + normals[0] * h;
                    Vector3 s1 = basePos1 + normals[1] * h;
                    Vector3 s2 = basePos2 + normals[2] * h;
                    float distance = Vector3.Dot(ray_dir, (1 - alpha - beta) * s0 + alpha * s1 + beta * s2 - ray_orig);
                    // draw linear ray
                    Debug.DrawLine(ray_orig + ray_dir * tPrevious, ray_orig + ray_dir * distance, color);

                    // draw nonlinear ray
                    Debug.DrawLine(uvwPrevious, uvw, color);

                    //Debug.Log("alpha: " + alpha);
                    //Debug.Log("beta: " + beta);
                    //Debug.Log("tPrevious: " + tPrevious + " t: " + distance + " tStart: " + tStart + " tEnd: " + tEnd);
                    //Debug.Log("h: " + h + " hStart: " + hStart + " hEnd: " + hEnd);
                    //Debug.Log("uvwPrevious: " + uvwPrevious + " uvw: " + uvw);
                    tPrevious = distance;
                    uvwPrevious = uvw;
                    color[0] -= 0.002f;
                    color[2] += 0.002f;
                    count++;
                }

                //Debug.Log("count: " + count);
            }
        }

        // Update is called once per frame
        public void Update()
        {
            ray_dir.Normalize();
            rend.material = mat;
            filt.mesh = BuildMesh();
            judgePrismIntersetion();
            if (inIntersect && outIntersect)
            {
                Debug.DrawLine(ray_orig, ray_orig + ray_dir * tin, Color.white);
                //Debug.DrawLine(ray_orig + ray_dir * tStart, ray_orig + ray_dir * tEnd, Color.magenta);
                Debug.DrawLine(ray_orig + ray_dir * tout, ray_orig + ray_dir * 5.0f, Color.white);
                transmittanceHDDA(0);
            }
            inIntersect = false;
            outIntersect = false;
        }
    } // class
} // namespace