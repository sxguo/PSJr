#include "common.h"
#include "scene.h"
#include "image.h"
#include "gls.h"

string scene_filename;  // scene filename
string image_filename;  // image filename
Scene* scene;           // scene arrays
//image3f bricktexture;

void uiloop();          // UI loop


// map used to uniquify edges
struct EdgeMap {
    map<pair<int,int>,int>  _edge_map;  // internal map
    vector<vec2i>           _edge_list; // internal list to generate unique ids

    // create an edge map for a collection of triangles and quads
    EdgeMap(vector<vec3i> triangle, vector<vec4i> quad) {
        for(auto f : triangle) { _add_edge(f.x,f.y); _add_edge(f.y,f.z); _add_edge(f.z,f.x); }
        for(auto f : quad) { _add_edge(f.x,f.y); _add_edge(f.y,f.z); _add_edge(f.z,f.w); _add_edge(f.w,f.x); }
    }

    // internal function to add an edge
    void _add_edge(int i, int j) {
        if(_edge_map.find(make_pair(i,j)) == _edge_map.end()) {
            _edge_map[make_pair(i,j)] = _edge_list.size();
            _edge_map[make_pair(j,i)] = _edge_list.size();
            _edge_list.push_back(vec2i(i,j));
        }
    }

    // edge list
    const vector<vec2i>& edges() const { return _edge_list; }

    // get an edge from two vertices
    int edge_index(vec2i e) const {
        error_if_not(not (_edge_map.find(make_pair(e.x,e.y)) == _edge_map.end()), "non existing edge");
        return _edge_map.find(make_pair(e.x, e.y))->second;
    }
};


// make normals for each face - duplicates all vertex data
void facet_normals(Mesh* mesh) {
    // allocates new arrays
    auto pos = vector<vec3f>();
    auto norm = vector<vec3f>();
    auto texcoord = vector<vec2f>();
    auto triangle = vector<vec3i>();
    auto quad = vector<vec4i>();

    // foreach triangle
    for(auto f : mesh->triangle) {
        // grab current pos size
        auto nv = (int)pos.size();
        // compute face face normal
        auto fn = normalize(cross(mesh->pos[f.y]-mesh->pos[f.x], mesh->pos[f.z]-mesh->pos[f.x]));
        // add triangle
        triangle.push_back({nv,nv+1,nv+2});
        // add vertex data
        for(auto i : range(3)) {
            pos.push_back(mesh->pos[f[i]]);
            norm.push_back(fn);
            if(not mesh->texcoord.empty()) texcoord.push_back(mesh->texcoord[f[i]]);
        }
    }

    // foreach quad
    for(auto f : mesh->quad) {
        // grab current pos size
        auto nv = (int)pos.size();
        // compute face normal
        auto fn = normalize(normalize(cross(mesh->pos[f.y]-mesh->pos[f.x], mesh->pos[f.z]-mesh->pos[f.x])) +
                            normalize(cross(mesh->pos[f.z]-mesh->pos[f.x], mesh->pos[f.w]-mesh->pos[f.x])));
        // add quad
        quad.push_back({nv,nv+1,nv+2,nv+3});
        // add vertex data
        for(auto i : range(4)) {
            pos.push_back(mesh->pos[f[i]]);
            norm.push_back(fn);
            if(not mesh->texcoord.empty()) texcoord.push_back(mesh->texcoord[f[i]]);
        }
    }

    // set back mesh data
    mesh->pos = pos;
    mesh->norm = norm;
    mesh->texcoord = texcoord;
    mesh->triangle = triangle;
    mesh->quad = quad;
}

// smooth out normal - does not duplicate data
void smooth_normals(Mesh* mesh) {
    // set normals array to the same length as pos and init all elements to zero
    mesh->norm.assign(mesh->pos.size(), zero3f);

    //put_your_code_here("Implement normal smoothing");

    // foreach triangle
    for(vec3i f : mesh->triangle){
        // compute face normal
        auto fn = normalize(cross(mesh->pos[f.y]-mesh->pos[f.x], mesh->pos[f.z]-mesh->pos[f.x]));
        // accumulate face normal to the vertex normals of each face index
        mesh->norm[f.x] += fn;
        mesh->norm[f.y] += fn;
        mesh->norm[f.z] += fn;

    }
    // foreach quad
    for(vec4i f : mesh->quad){
        // compute face normal
        auto fn = normalize(cross(mesh->pos[f.y]-mesh->pos[f.x], mesh->pos[f.z]-mesh->pos[f.x]));
        // accumulate face normal to the vertex normals of each face index
        mesh->norm[f.x] += fn;
        mesh->norm[f.y] += fn;
        mesh->norm[f.z] += fn;
        mesh->norm[f.w] += fn;
    }
    // normalize all vertex normals
    for (auto& t : mesh->norm) t = normalize(t);
}

// smooth out tangents
void smooth_tangents(Mesh* polyline) {
    // set tangent array
    polyline->norm = vector<vec3f>(polyline->pos.size(),zero3f);
    // foreach line
    for(auto l : polyline->line) {
        // compute line tangent
        auto lt = normalize(polyline->pos[l.y]-polyline->pos[l.x]);
        // accumulate segment tangent to vertex tangent on each vertex
        for (auto i : range(2)) polyline->norm[l[i]] += lt;
    }
    // normalize all vertex tangents
    for (auto& t : polyline->norm) t = normalize(t);
}

// subdivide bezier spline into line segments (assume bezier has only bezier segments and no lines)
// subdivide using uniform sampling
void subdivide_bezier_uniform(Mesh *bezier) {
    auto pos = vector<vec3f>();
    auto line = vector<vec2i>();

    // determine number of steps
    int steps = 1 << bezier->subdivision_bezier_level;

    //put_your_code_here("Implement uniform Bezier spline subdivision");

    // foreach spline segment
    for(auto ss : bezier->spline){
        // get control points of segment
        auto cp0 = bezier->pos[ss.x];
        auto cp1 = bezier->pos[ss.y];
        auto cp2 = bezier->pos[ss.z];
        auto cp3 = bezier->pos[ss.w];

        // note the starting index of new points
        int newInd = pos.size();

        // foreach step
        for(int step=0; step<=steps; step++){
            if(step==0 && pos.size()<=0)
                newInd = 0;
            // compute t for current segment
            auto t = (step*1.0)/steps;
            // compute blending weights
            auto b0 = bernstein(t, 0, 3);
            auto b1 = bernstein(t, 1, 3);
            auto b2 = bernstein(t, 2, 3);
            auto b3 = bernstein(t, 3, 3);

            // compute new point position
            auto b = b0*cp0 + b1*cp1 + b2*cp2 + b3*cp3;
            // add new point to pos vector
            pos.push_back(vec3f(b));
        }
        // foreach step
        for(int step=0; step<steps; step++){
            // create line segment
            line.push_back(vec2i(step+newInd,step+newInd+1));
        }
    }
    // copy vertex positions
    bezier->pos = pos;
    // copy line segments
    bezier->line = line;

    // clear bezier array from lines
    bezier->spline.clear();
    bezier->subdivision_bezier_level = 0;

    // run smoothing to get proper tangents
    smooth_tangents(bezier);
}

// subdivide bezier spline into line segments (assume bezier has only bezier segments and no lines)
// subdivide using de casteljau algorithm
void subdivide_bezier_decasteljau(Mesh *bezier) {
    auto pos = bezier->pos;
    auto splines = bezier->spline;

    //put_your_code_here("Implement de Casteljau algorithm");

    // *note*: this psuedocode is for an iterative implementation of the algorithm without adaptive subd
    // foreach level
    for(int i=0;i<bezier->subdivision_bezier_level;i++){
        // make new arrays of positions and bezier segments
        vector<vec3f> ipos = vector<vec3f>();
        vector<vec4i> isplines = vector<vec4i>();

        // copy all the vertices into the new array (this wastes space but it is easier for now)
        for(vec3f v : pos)
            ipos.push_back(v);

        //foreach bezier segment
        for(vec4i ss : splines){
            // get control points of segment
            vec3f p0 = ipos[ss.x]; // ss.x = index in pos
            vec3f p1 = ipos[ss.y];
            vec3f p2 = ipos[ss.z];
            vec3f p3 = ipos[ss.w];

            // note the starting index of new points
            int posInd = ss.x;      // posInd gives index of first control point
            int mInd = ipos.size(); // mInd gives END of ipos to know where to add midpoints

            // first level midpoints
            vec3f q0 = (p0+p1)/2.0;
            vec3f q1 = (p1+p2)/2.0;
            vec3f q2 = (p2+p3)/2.0;
            // second level midpoints
            vec3f r0 = (q0+q1)/2.0;
            vec3f r1 = (q1+q2)/2.0;
            // third level midpoint
            vec3f s = (r0+r1)/2.0;

            // calculate midpoints and new indices
            int q0_idx = ipos.size();
            ipos.push_back(q0);
            int q2_idx = ipos.size();
            ipos.push_back(q2);
            int r0_idx = ipos.size();
            ipos.push_back(r0);
            int r1_idx = ipos.size();
            ipos.push_back(r1);
            int s_idx = ipos.size();
            ipos.push_back(s);

            // push new control points into isplines
            isplines.push_back(vec4i(posInd,q0_idx,r0_idx,s_idx));
            isplines.push_back(vec4i(s_idx,r1_idx,q2_idx,ss.w));
        }
        // set new arrays pos, segments into the working lineset
        pos = ipos;
        splines = isplines;
    }
    // copy vertex positions
    bezier->pos = pos;

    // copy bezier segments into line segments
    bezier->line.clear();
    for(auto spline : splines) bezier->line.push_back({spline.x,spline.w});

    // clear bezier array from lines
    bezier->spline.clear();
    bezier->subdivision_bezier_level = 0;

    // run smoothing to get proper tangents
    smooth_tangents(bezier);
}

// subdivide bezier spline into line segments (assume bezier has only bezier segments and no lines)
void subdivide_bezier(Mesh* bezier) {
    // skip is needed
    if(not bezier->subdivision_bezier_level) return;

    if(bezier->subdivision_bezier_uniform) subdivide_bezier_uniform(bezier);
    else subdivide_bezier_decasteljau(bezier);
}



// apply Catmull-Clark mesh subdivision
// does not subdivide texcoord
void subdivide_catmullclark(Mesh* subdiv) {
    // skip is needed
    if(not subdiv->subdivision_catmullclark_level) return;

    // allocate a working Mesh copied from the subdiv
    auto mesh = new Mesh(*subdiv);
    // foreach level
    for(int i=0; i<subdiv->subdivision_catmullclark_level; i++) {

        // make empty pos and quad arrays
        auto pos = vector<vec3f>();
        auto quad = vector<vec4i>();

        // create edge_map from current mesh
        auto edge_map = EdgeMap(mesh->triangle,mesh->quad);

        //put_your_code_here("Implement Catmull-Clark Subdivision");

        // linear subdivision - create vertices --------------------------------------
        // copy all vertices from the current mesh
        for(vec3f v : mesh->pos){
            pos.push_back(v);
        }
        int eo = pos.size(); // edge offset

        // add vertices in the middle of each edge (use EdgeMap)
        for(vec2i e : edge_map.edges()){
            // if edge is not already in pos, add to pos
            pos.push_back(mesh->pos[e.x]*0.5+mesh->pos[e.y]*0.5);
        }
        int to = pos.size(); // triangle offset

        // add face vertices in the middle of each triangle
        for(vec3i f : mesh->triangle)
            pos.push_back((mesh->pos[f.x]+mesh->pos[f.y]+mesh->pos[f.z])/3);
        int qo = pos.size(); // quad offset

        // add face vertices in the middle of each quad
        for(vec4i f : mesh->quad)
            pos.push_back(mesh->pos[f.x]*0.25+mesh->pos[f.y]*0.25+mesh->pos[f.z]*0.25+mesh->pos[f.w]*0.25);

        // subdivision pass ----------------------------------------------------------
        // compute an offset for the edge vertices (see int eo)
        // compute an offset for the triangle vertices (see int to)
        // compute an offset for the quad vertices (see int qo)

        // foreach triangle
        for(int ct=0;ct<mesh->triangle.size();ct++) {
            vec3i f = mesh->triangle[ct];  // f contains index of 3verts of curr tri in mesh->triangle
            vec3i ve = vec3i(edge_map.edge_index(vec2i(f.x, f.y))+eo,   //ve.x
                             edge_map.edge_index(vec2i(f.y, f.z))+eo,   //ve.y
                             edge_map.edge_index(vec2i(f.z, f.x))+eo);  //ve.z
            // face vertex index is index of current triangle + triangle offset
            int fv = ct+to;
            //add three quads to the new quad array
            quad.push_back(vec4i(f.x,ve.x,fv,ve.z));
            quad.push_back(vec4i(f.y,ve.y,fv,ve.x));
            quad.push_back(vec4i(f.z,ve.z,fv,ve.y));
        }
        // foreach quad
        for(int cq=0;cq<mesh->quad.size();cq++) {
            vec4i f = mesh->quad[cq];
            vec4i ve = vec4i(edge_map.edge_index(vec2i(f.x, f.y))+eo,
                             edge_map.edge_index(vec2i(f.y, f.z))+eo,
                             edge_map.edge_index(vec2i(f.z, f.w))+eo,
                             edge_map.edge_index(vec2i(f.w, f.x))+eo);
            // face vertex index is index of current quad + quad offset
            auto fv = cq+qo;
            // add four quads to the new quad array
            quad.push_back(vec4i(f.x,ve.x,fv,ve.w));
            quad.push_back(vec4i(f.y,ve.y,fv,ve.x));
            quad.push_back(vec4i(f.z,ve.z,fv,ve.y));
            quad.push_back(vec4i(f.w,ve.w,fv,ve.z));
        }

        // averaging pass ------------------------------------------------------------
        // create arrays to compute pos averages (avg_pos, avg_count)
        // arrays have the same length as the new pos array, and are init to zero
        vector<vec3f> avg_pos = vector<vec3f>();
        int avg_count[pos.size()];
        for(int i=0; i<pos.size();i++){
            avg_pos.push_back(zero3f);
            avg_count[i] = 0;
        }

        // for each new quad
        for(vec4i q : quad){
            // compute quad center using the new pos array
            vec3f qc = (pos[q.x]+pos[q.y]+pos[q.z]+pos[q.w])*0.25;

            // foreach vertex index in the quad
            // accumulate face center to the avg_pos and add 1 to avg_count
            avg_pos[q.x] += qc;
            avg_pos[q.y] += qc;
            avg_pos[q.z] += qc;
            avg_pos[q.w] += qc;
            avg_count[q.x] += 1;
            avg_count[q.y] += 1;
            avg_count[q.z] += 1;
            avg_count[q.w] += 1;
        }
        // normalize avg_pos with its count avg_count
        for(int a=0; a<avg_pos.size(); a++){
            avg_pos[a] = avg_pos[a] / ((float)avg_count[a]);
        }

        // correction pass -----------------------------------------------------------
        // foreach pos, compute correction p = p + (avg_p - p) * (4/avg_count)
        for(int p=0; p<pos.size(); p++){
            pos[p] = pos[p] + ((avg_pos[p] - pos[p]) * (4.0/avg_count[p]));
        }

        // set new arrays pos, quad back into the working mesh; clear triangle array
        mesh->pos = pos;
        mesh->triangle = vector<vec3i>();
        mesh->quad = quad;
    }

    // clear subdivision
    mesh->subdivision_catmullclark_level = 0;

    // according to smooth, either smooth_normals or facet_normals
    if(subdiv->subdivision_catmullclark_smooth) smooth_normals(mesh);
    else facet_normals(mesh);

    // copy back
    *subdiv = *mesh;

    // clear
    delete mesh;
}

void subdivide_surface(Surface* surface) {
    // create mesh struct
    auto mesh    = new Mesh{};
    // copy frame
    mesh->frame  = surface->frame;
    // copy material
    mesh->mat    = surface->mat;

    // get surface radius
    auto radius  = surface->radius;

    // vertexidx is used to look up index of vertex corresponding to (i,j)
    map<pair<int,int>,int> vertexidx;

    if(surface->isquad) {
        // compute how much to subdivide
        auto ci = 1 << surface->subdivision_level;
        auto cj = 1 << surface->subdivision_level;

        // compute corners of quad
        auto p00 = vec3f(-1,-1,0) * radius;
        auto p01 = vec3f(-1, 1,0) * radius;
        auto p10 = vec3f( 1,-1,0) * radius;
        auto p11 = vec3f( 1, 1,0) * radius;

        // foreach column
        for(auto i : range(ci+1)) {
            // foreach row
            for(auto j : range(cj+1)) {
                // compute u,v corresponding to column and row
                auto u = i / (float)ci;
                auto v = j / (float)cj;

                // compute new point location
                auto p = p00*u*v + p01*u*(1-v) + p10*(1-u)*v + p11*(1-u)*(1-v);

                // insert point into pos vector, remembering its index
                vertexidx[make_pair(i,j)] = mesh->pos.size();
                mesh->pos.push_back(p);
                mesh->norm.push_back(z3f);
            }
        }
        // foreach column
        for(auto i : range(ci)) {
            // foreach row
            for(auto j : range(cj)) {
                // find indices of neigboring vertices
                int idx0 = vertexidx[make_pair(i+0,j+0)];
                int idx1 = vertexidx[make_pair(i+1,j+0)];
                int idx2 = vertexidx[make_pair(i+1,j+1)];
                int idx3 = vertexidx[make_pair(i+0,j+1)];

                // create quad
                mesh->quad.push_back({idx0, idx1, idx2, idx3});
            }
        }
    }/* else if(surface->isCube){
        int radius = surface->radius;
        float offset = 0.0;

        mesh->pos = { {-radius,-radius,-radius}, {-radius,-radius,radius}, {-radius,radius,-radius}, {-radius,radius,radius},
            {radius,-radius,-radius}, {radius,-radius,radius}, {radius,radius,-radius}, {radius,radius,radius}};
        mesh->norm = {z3f,z3f,z3f,z3f,z3f,z3f};
        mesh->quad = { {0,1,3,2}, {0,1,5,4}, {4,5,7,6}, {6,7,3,2}, {0,2,4,6}, {1,3,7,5} };

    } */
    else {
        //put_your_code_here("Implement sphere subdivision");

        // compute how much to subdivide
        auto ci = 1 << (surface->subdivision_level+2); // 2 pow (l+2)
        auto cj = 1 << (surface->subdivision_level+1); // 2 pow (l+1)

        // foreach column
        for(auto i : range(ci)) {
            // foreach row
            for(auto j : range(cj+1)) {
                // compute phi,theta for column and row
                float phi = i/(float)ci*2*pif;
                float theta = j/(float)cj*pif;
                // compute new point location
                auto px = radius*cos(phi)*sin(theta);
                auto py = radius*sin(phi)*sin(theta);
                auto pz = radius*cos(theta);
                auto p = vec3f(px,py,pz);
                // insert point into pos vector, remembering its index
                vertexidx[make_pair(i,j)] = mesh->pos.size();
                mesh->pos.push_back(p*radius);
                mesh->norm.push_back(p);
            }
        }
        // foreach column
        for(int i : range(ci)) {
            // foreach row
            for(int j : range(cj)) {

                // create triangle (face touching pole) or quad
                // if first row of sphere, create upright triangle
                if(j==0){
                    int idx0 = vertexidx[make_pair(0,0)];
                    int idx1 = vertexidx[make_pair(i,j+1)];
                    int idx2 = vertexidx[make_pair((i+1)%ci,j+1)];
                    mesh->triangle.push_back({idx0, idx1, idx2});
                }
                // if last row of sphere, create upside down triangle
                else if(j==cj-1){
                    int idx0 = vertexidx[make_pair(i+0,j+0)];
                    int idx1 = vertexidx[make_pair(0,j+1)];
                    int idx2 = vertexidx[make_pair((i+1)%ci,j)];
                    mesh->triangle.push_back({idx0, idx1, idx2});
                }
                // if in any other row, create quad
                else{
                    int idx0 = vertexidx[make_pair(i+0,j+0)];
                    int idx1 = vertexidx[make_pair(i+0,j+1)];
                    int idx2 = vertexidx[make_pair((i+1)%ci,j+1)];
                    int idx3 = vertexidx[make_pair((i+1)%ci,j+0)];
                    mesh->quad.push_back({idx0, idx1, idx2, idx3});
                }
            }
        }
    }

    // according to smooth, either smooth_normals or facet_normals
    if(surface->subdivision_smooth) smooth_normals(mesh);
    else facet_normals(mesh);

    // update _display_mesh of surface
    surface->_display_mesh = mesh;
}

void traverse(Surface* surface){

    // create transform matrix currxform for current surface
    // this matrix will be pushed into xformlist
    mat4f smatrix = scaling_matrix(surface->sform);
    mat4f tmatrix = translation_matrix(surface->tform);

    float angle_z = surface->rform.z;
    float angle_y = surface->rform.y;
    float angle_x = surface->rform.x;
    mat4f rotate_z = rotation_matrix(angle_z,surface->frame.z);
    mat4f rotate_y = rotation_matrix(angle_y,surface->frame.y);
    mat4f rotate_x = rotation_matrix(angle_x,surface->frame.x);
    mat4f rmatrix = rotate_z*rotate_y*rotate_x;

    mat4f currxform = tmatrix*rmatrix*smatrix;

    // if has children, set children's parentframe variable
    if(surface->hasChildren){
        for(int c=0; c<surface->children.size(); c++){
            Surface* child = scene->surfaces[surface->children[c]];
            child->parentFrame = surface->frame;

            // if applyColor is true, change children's color to surface color
            if(surface->applyColor){
                child->mat->kd = surface->mat->kd;
            }
        }
    }

    // if not the root node (has parent), add transformation of [child to parent] and [xform]
    if(!surface->isRoot){
        // find transformation matrix from child to parent frame
        mat4f parentFrame = frame_to_matrix(surface->parentFrame);
        mat4f currFrame = frame_to_matrix(surface->frame);
        mat4f childToParent = inverse(currFrame)*parentFrame;

        currxform = currxform*childToParent;
    }

    // push(mlocal) onto the end of xformlist
    scene->xformlist.push_back(currxform);

    // reset currxform
    currxform = identity_mat4f;

    // calculate final transform matrix xform from xformlist
    for(mat4f xform : scene->xformlist){
        currxform = currxform*xform;
    }

    // call traverse on each child surface
    for(int c=0; c<surface->children.size(); c++){
        Surface* child = scene->surfaces[surface->children[c]];
        traverse(child);
    }

    // transform surface's pos vec3f by currxform
    for(int pInd=0; pInd<surface->_display_mesh->pos.size(); pInd++){
        vec3f pos = surface->_display_mesh->pos[pInd];
        vec3f p = transform_point(currxform, pos);

        surface->_display_mesh->pos[pInd] = p;
    }

    // transform surface's norm vec3f by currxform
    for(int nInd=0; nInd<surface->_display_mesh->norm.size(); nInd++){
        vec3f norm = surface->_display_mesh->norm[nInd];
        vec3f n = transform_normal(currxform,norm);

        surface->_display_mesh->norm[nInd] = n;
    }

    // pop last matrix in xformlist
    scene->xformlist.pop_back();
}

void subdivide(Scene* scene) {
    for(auto mesh : scene->meshes) {
        if(mesh->subdivision_catmullclark_level) subdivide_catmullclark(mesh);
        if(mesh->subdivision_bezier_level) subdivide_bezier(mesh);
    }
    for(int s=0; s<scene->surfaces.size(); s++) {
        subdivide_surface(scene->surfaces[s]);

        // if the surface is a root, add its index to rootlist
        if(scene->surfaces[s]->isRoot){
            scene->rootlist.push_back(s);
        }
    }
    // call traverse on all root nodes
    for(int root=0; root<scene->rootlist.size(); root++){
        int rootInd = scene->rootlist[root];
        traverse(scene->surfaces[rootInd]);
    }
}

// main function
int main(int argc, char** argv) {
    auto args = parse_cmdline(argc, argv,
        { "02_model", "view scene",
            {  {"resolution",     "r", "image resolution", typeid(int),    true,  jsonvalue() }  },
            {  {"scene_filename", "",  "scene filename",   typeid(string), false, jsonvalue("scene.json")},
               {"image_filename", "",  "image filename",   typeid(string), true,  jsonvalue("")}  }
        });

    // generate/load scene either by creating a test scene or loading from json file
    scene_filename = args.object_element("scene_filename").as_string();
    scene = nullptr;
    if(scene_filename.length() > 9 and scene_filename.substr(0,9) == "testscene") {
        int scene_type = atoi(scene_filename.substr(9).c_str());
        scene = create_test_scene(scene_type);
        scene_filename = scene_filename + ".json";
    } else {
        scene = load_json_scene(scene_filename);
    }
    error_if_not(scene, "scene is nullptr");

    image_filename = (args.object_element("image_filename").as_string() != "") ?
        args.object_element("image_filename").as_string() :
        scene_filename.substr(0,scene_filename.size()-5)+".png";

    if(not args.object_element("resolution").is_null()) {
        scene->image_height = args.object_element("resolution").as_int();
        scene->image_width = scene->camera->width * scene->image_height / scene->camera->height;
    }
    subdivide(scene);

    uiloop();
}




/////////////////////////////////////////////////////////////////////
// UI and Rendering Code: OpenGL, GLFW, GLSL


int gl_program_id         = 0;  // OpenGL program handle
int gl_vertex_shader_id   = 0;  // OpenGL vertex shader handle
int gl_fragment_shader_id = 0;  // OpenGL fragment shader handle
map<image3f*,int> gl_texture_id;// OpenGL texture handles

bool save      = false;         // whether to start the save loop
bool wireframe = false;         // display as wireframe

void init_shaders();            // initialize the shaders
void init_textures();           // initialize the textures
void shade();                   // render the scene with OpenGL
void _shade_mesh(Mesh* mesh);
void character_callback(GLFWwindow* window, unsigned int key);  // ...
                                // glfw callback for character input
void _bind_texture(string name_map, string name_on, image3f* txt, int pos); // ...
                                // utility to bind texture parameters for shaders
                                // uses texture name, texture_on name, texture pointer and texture unit position

// glfw callback for character input
void character_callback(GLFWwindow* window, unsigned int key) {
    if(key == 's') save = true;
    if(key == 'w') wireframe = not wireframe;
}

// uiloop
void uiloop() {
    auto ok_glfw = glfwInit();
    error_if_not(ok_glfw, "glfw init error");

    glfwWindowHint(GLFW_SAMPLES, scene->image_samples);

    auto window = glfwCreateWindow(scene->image_width,
                                   scene->image_height,
                                   "graphics13 | model", NULL, NULL);
    error_if_not(window, "glfw window error");

    glfwMakeContextCurrent(window);

    glfwSetCharCallback(window, character_callback);
    glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_NORMAL);

    auto ok_glew = glewInit();
    error_if_not(GLEW_OK == ok_glew, "glew init error");

    init_shaders();
    init_textures();

    auto mouse_last_x = -1.0;
    auto mouse_last_y = -1.0;

    while(not glfwWindowShouldClose(window)) {
        glfwGetFramebufferSize(window, &scene->image_width, &scene->image_height);
        scene->camera->width = (scene->camera->height * scene->image_width) / scene->image_height;

        shade();

        if(glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT)) {
            double x, y;
            glfwGetCursorPos(window, &x, &y);
            if (mouse_last_x < 0 or mouse_last_y < 0) { mouse_last_x = x; mouse_last_y = y; }
            auto delta_x = x - mouse_last_x, delta_y = y - mouse_last_y;

            set_view_turntable(scene->camera, delta_x*0.01, -delta_y*0.01, 0, 0, 0);

            mouse_last_x = x;
            mouse_last_y = y;
        } else { mouse_last_x = -1; mouse_last_y = -1; }

        if(save) {
            auto image = image3f(scene->image_width,scene->image_height);
            glReadPixels(0, 0, scene->image_width, scene->image_height, GL_RGB, GL_FLOAT, &image.at(0,0));
            write_png(image_filename, image, true);
            save = false;
        }

        glfwSwapBuffers(window);
        glfwPollEvents();
    }

    glfwDestroyWindow(window);

    glfwTerminate();
}

// initialize the shaders
void init_shaders() {
    // load shader code from files
    auto vertex_shader_code    = load_text_file("model_vertex.glsl");
    auto fragment_shader_code  = load_text_file("model_fragment.glsl");
    auto vertex_shader_codes   = (char *)vertex_shader_code.c_str();
    auto fragment_shader_codes = (char *)fragment_shader_code.c_str();

    // create shaders
    gl_vertex_shader_id   = glCreateShader(GL_VERTEX_SHADER);
    gl_fragment_shader_id = glCreateShader(GL_FRAGMENT_SHADER);

    // load shaders code onto the GPU
    glShaderSource(gl_vertex_shader_id,1,(const char**)&vertex_shader_codes,nullptr);
    glShaderSource(gl_fragment_shader_id,1,(const char**)&fragment_shader_codes,nullptr);

    // compile shaders
    glCompileShader(gl_vertex_shader_id);
    glCompileShader(gl_fragment_shader_id);

    // check if shaders are valid
    error_if_glerror();
    error_if_shader_not_valid(gl_vertex_shader_id);
    error_if_shader_not_valid(gl_fragment_shader_id);

    // create program
    gl_program_id = glCreateProgram();

    // attach shaders
    glAttachShader(gl_program_id,gl_vertex_shader_id);
    glAttachShader(gl_program_id,gl_fragment_shader_id);

    // bind vertex attributes locations
    glBindAttribLocation(gl_program_id, 0, "vertex_pos");
    glBindAttribLocation(gl_program_id, 1, "vertex_norm");
    glBindAttribLocation(gl_program_id, 2, "vertex_texcoord");

    // link program
    glLinkProgram(gl_program_id);

    // check if program is valid
    error_if_glerror();
    error_if_program_not_valid(gl_program_id);
}

// initialize the textures
void init_textures() {
    // grab textures from scene
    auto textures = get_textures(scene);
    // foreach texture
    for(auto texture : textures) {
        // if already in the gl_texture_id map, skip
        if(gl_texture_id.find(texture) != gl_texture_id.end()) continue;
        // gen texture id
        unsigned int id = 0;
        glGenTextures(1, &id);
        // set id to the gl_texture_id map for later use
        gl_texture_id[texture] = id;
        // bind texture
        glBindTexture(GL_TEXTURE_2D, id);
        // set texture filtering parameters
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_GENERATE_MIPMAP, GL_TRUE);
        // load texture data
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA,
                     texture->width(), texture->height(),
                     0, GL_RGB, GL_FLOAT, texture->data());
    }
}


// utility to bind texture parameters for shaders
// uses texture name, texture_on name, texture pointer and texture unit position
void _bind_texture(string name_map, string name_on, image3f* txt, int pos) {
    // if txt is not null
    if(txt) {
        // set texture on boolean parameter to true
        glUniform1i(glGetUniformLocation(gl_program_id,name_on.c_str()),GL_TRUE);
        // activate a texture unit at position pos
        glActiveTexture(GL_TEXTURE0+pos);
        // bind texture object to it from gl_texture_id map
        glBindTexture(GL_TEXTURE_2D, gl_texture_id[txt]);
        // set texture parameter to the position pos
        glUniform1i(glGetUniformLocation(gl_program_id, name_map.c_str()), pos);
    } else {
        // set texture on boolean parameter to false
        glUniform1i(glGetUniformLocation(gl_program_id,name_on.c_str()),GL_FALSE);
        // activate a texture unit at position pos
        glActiveTexture(GL_TEXTURE0+pos);
        // set zero as the texture id
        glBindTexture(GL_TEXTURE_2D, 0);
    }
}

// render the scene with OpenGL
void shade() {
    // enable depth test
    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LEQUAL);
    // disable culling face
    glDisable(GL_CULL_FACE);
    // let the shader control the points
    glEnable(GL_POINT_SPRITE);

    // set up the viewport from the scene image size
    glViewport(0, 0, scene->image_width, scene->image_height);

    // clear the screen (both color and depth) - set cleared color to background
    glClearColor(scene->background.x, scene->background.y, scene->background.z, 1);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    // enable program
    glUseProgram(gl_program_id);

    // bind camera's position, inverse of frame and projection
    // use frame_to_matrix_inverse and frustum_matrix
    glUniform3fv(glGetUniformLocation(gl_program_id,"camera_pos"),
                 1, &scene->camera->frame.o.x);
    glUniformMatrix4fv(glGetUniformLocation(gl_program_id,"camera_frame_inverse"),
                       1, true, &frame_to_matrix_inverse(scene->camera->frame)[0][0]);
    glUniformMatrix4fv(glGetUniformLocation(gl_program_id,"camera_projection"),
                       1, true, &frustum_matrix(-scene->camera->dist*scene->camera->width/2, scene->camera->dist*scene->camera->width/2,
                                                -scene->camera->dist*scene->camera->height/2, scene->camera->dist*scene->camera->height/2,
                                                scene->camera->dist,10000)[0][0]);

    // bind ambient and number of lights
    glUniform3fv(glGetUniformLocation(gl_program_id,"ambient"),1,&scene->ambient.x);
    glUniform1i(glGetUniformLocation(gl_program_id,"lights_num"),scene->lights.size());

    // foreach light
    auto count = 0;
    for(auto light : scene->lights) {
        // bind light position and internsity (create param name with tostring)
        glUniform3fv(glGetUniformLocation(gl_program_id,tostring("light_pos[%d]",count).c_str()),
                     1, &light->frame.o.x);
        glUniform3fv(glGetUniformLocation(gl_program_id,tostring("light_intensity[%d]",count).c_str()),
                     1, &light->intensity.x);
        count++;
    }

    // foreach mesh
//    for(auto mesh : scene->meshes) {
//        _shade_mesh(mesh);
//    }

    for(auto surf : scene->surfaces) {
        _shade_mesh(surf->_display_mesh);
    }
}

void _shade_mesh(Mesh* mesh) {
    // bind material kd, ks, n
    ERROR_IF_NOT(mesh, "mesh is null");
    glUniform3fv(glGetUniformLocation(gl_program_id,"material_kd"),1,&mesh->mat->kd.x);
    glUniform3fv(glGetUniformLocation(gl_program_id,"material_ks"),1,&mesh->mat->ks.x);
    glUniform1f(glGetUniformLocation(gl_program_id,"material_n"),mesh->mat->n);

    // bind texture params (txt_on, sampler)
    _bind_texture("material_kd_txt",   "material_kd_txt_on",   mesh->mat->kd_txt,   0);
    _bind_texture("material_ks_txt",   "material_ks_txt_on",   mesh->mat->ks_txt,   1);
    _bind_texture("material_norm_txt", "material_norm_txt_on", mesh->mat->norm_txt, 2);

    // bind mesh frame - use frame_to_matrix
    glUniformMatrix4fv(glGetUniformLocation(gl_program_id,"mesh_frame"),1,true,&frame_to_matrix(mesh->frame)[0][0]);

    // enable vertex attributes arrays and set up pointers to the mesh data
    auto vertex_pos_location = glGetAttribLocation(gl_program_id, "vertex_pos");
    auto vertex_norm_location = glGetAttribLocation(gl_program_id, "vertex_norm");
    auto vertex_texcoord_location = glGetAttribLocation(gl_program_id, "vertex_texcoord");
    glEnableVertexAttribArray(vertex_pos_location);
    glVertexAttribPointer(vertex_pos_location, 3, GL_FLOAT, GL_FALSE, 0, &mesh->pos[0].x);
    glEnableVertexAttribArray(vertex_norm_location);
    glVertexAttribPointer(vertex_norm_location, 3, GL_FLOAT, GL_FALSE, 0, &mesh->norm[0].x);
    if(not mesh->texcoord.empty()) {
        glEnableVertexAttribArray(vertex_texcoord_location);
        glVertexAttribPointer(vertex_texcoord_location, 2, GL_FLOAT, GL_FALSE, 0, &mesh->texcoord[0].x);
    }
    else glVertexAttrib2f(vertex_texcoord_location, 0, 0);

    // draw triangles and quads
    if(not wireframe) {
        if(mesh->triangle.size()) glDrawElements(GL_TRIANGLES, mesh->triangle.size()*3, GL_UNSIGNED_INT, &mesh->triangle[0].x);
        if(mesh->quad.size()) glDrawElements(GL_QUADS, mesh->quad.size()*4, GL_UNSIGNED_INT, &mesh->quad[0].x);
    } else {
        auto edges = EdgeMap(mesh->triangle, mesh->quad).edges();
        glDrawElements(GL_LINES, edges.size()*2, GL_UNSIGNED_INT, &edges[0].x);
    }

    // draw line sets
    if(not mesh->line.empty()) glDrawElements(GL_LINES, mesh->line.size()*2, GL_UNSIGNED_INT, mesh->line.data());
    for(auto segment : mesh->spline) glDrawElements(GL_LINE_STRIP, 4, GL_UNSIGNED_INT, &segment);

    // disable vertex attribute arrays
    glDisableVertexAttribArray(vertex_pos_location);
    glDisableVertexAttribArray(vertex_norm_location);
    if(not mesh->texcoord.empty()) glDisableVertexAttribArray(vertex_texcoord_location);
}
