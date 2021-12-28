// TestUltraMesh.cpp : This file contains the 'main' function. Program execution begins and ends there.
//

#include "pch.h"
#include <iostream>
#include <chrono>
#include "../UltraMesh.h"
#include "../window.h"
#include "pg/pgrender.h"
#include "C:/Eigen3.3.7/Eigen/Dense"


#define PROMPT(_str) PgWindowText(_str);
#define JOURNAL_DEBUG 0
#define MINIMAL_WALL_THICKNESS 2
#define VOXEL_SIZE 0.25
#define REMESH false
#define TIME_INTERVAL(end, start) double(std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count()) / 1000.0


PTSolid ReadSTL(PTEnvironment env, std::wstring fileName)
{
	PTStream stream = PV_ENTITY_NULL;
	PTStatus status = PV_STATUS_OK;

	std::string sFileName = std::string(fileName.cbegin(), fileName.cend());
	status += PFStreamFileOpen(PTString(sFileName.c_str()), PV_FILE_READ, NULL, &stream);

	PTSolidReadOpts solidReadOptions;
	PMInitSolidReadOpts(&solidReadOptions);
	solidReadOptions.file_colour_format = PV_FILE_COLOUR_STL_RGB;

	PTSolid model = PV_ENTITY_NULL;
	status += PFSolidRead(env, PV_SOLID_DATA_STL, stream, &solidReadOptions, &model);
	PFStreamClose(stream);
	return model;
}

void WriteSTL(PTSolid solid, std::wstring fileName)
{
    PTStream stream = PV_ENTITY_NULL;
    PTStatus status = PV_STATUS_OK;
    std::string sFileName = std::string(fileName.cbegin(), fileName.cend());
    status = PFStreamFileOpen(PTString(sFileName.c_str()), PV_FILE_WRITE, NULL, &stream);

    PTSolidWriteOpts solidWriteOptions;
    PMInitSolidWriteOpts(&solidWriteOptions);
    /* The solid has already been prepared for single-precision output */
    solidWriteOptions.prepare_for_single_precision = TRUE;

    status = PFSolidWrite(solid, PV_SOLID_DATA_BINARY_STL, stream, &solidWriteOptions);
     PFStreamClose(stream);
 }

PTSolid UltraMesh2PTSolid(UltraMesh& mesh, PTEnvironment env)
{
	PTSolid newSolid = PV_ENTITY_NULL;
	PTStatus status = PV_STATUS_OK;
	PTBeginSolidOpts BeginSolidOptions;
	PMInitBeginSolidOpts(&BeginSolidOptions);
	double Triangle[3][3];
	PTPointer Trig;

	Trig = &Triangle;
	status = PFSolidCreateBegin(env, PV_SOLID_CREATE_TRIANGLES, &BeginSolidOptions, &newSolid);
	for (auto face : mesh.m_faces)
	{
		Eigen::Vector3i idx = face.Vertices();
		for (int i = 0; i < 3; i++)
		{
			Eigen::Vector3d position = mesh.m_vertices[idx[i]].Position();
			memcpy(&Triangle[i][0], &position, sizeof(PTPoint));
		}
		status += PFSolidCreateTriangle(newSolid, Trig, NULL);
	}
	status += PFSolidCreateEnd(newSolid, NULL);
	if (status != PV_STATUS_OK)
		newSolid = 0;
	return newSolid;
}

void PTSolid2UltraMesh(PTSolid solid, UltraMesh& mesh)
{
	PTStatus status = PV_STATUS_OK;
	PTDirectSolidQuery query = PV_ENTITY_NULL;
	PTNat32 numFaces = 0;
	PTNat32 numVertices = 0;
	PTNat32 numEdges = 0;


	status |= PFSolidQueryDirectIndexed(solid, PV_MESH_TRIANGLES, NULL, &query);

	status |= PFDirectSolidQuerySingle(query, PV_DSQUERY_NUM_FACES, PV_DATA_TYPE_SIGNED_INT32, &numFaces, 0);

	status |= PFDirectSolidQuerySingle(query, PV_DSQUERY_NUM_EDGES, PV_DATA_TYPE_SIGNED_INT32, &numEdges, 0);

	status |= PFDirectSolidQuerySingle(query, PV_DSQUERY_NUM_VERTICES, PV_DATA_TYPE_SIGNED_INT32, &numVertices, 0);

	printf("Read solid with %d Faces, %d Vertices and %d Edges\n", numFaces, numVertices, numEdges);

	Eigen::Vector3d vertex;
	Eigen::Vector3i face;
	double newV[3];

	for (int idx = 0; idx < int(numVertices); idx++)
	{
		status |= PFDirectSolidQuerySingle(query, PV_DSQUERY_VERTEX_COORDS, PV_DATA_TYPE_DOUBLE, &vertex, idx);
		for (int i = 0; i < 3; i++)
			newV[i] = vertex[i];
        UltraVertex newVertex = UltraVertex(Eigen::Vector3d(newV));
        newVertex.m_index = mesh.m_vertices.size();
		mesh.m_vertices.push_back(newVertex);
	}

	for (int idx = 0; idx < int(numFaces); idx++)
	{
		PFDirectSolidQuerySingle(query, PV_DSQUERY_FACE_VERTICES, PV_DATA_TYPE_UNSIGNED_INT32, &face, idx);
		Eigen::Vector3d p;
		Eigen::Vector3d pp[3];
		for (int i = 0; i < 3; i++)
		{
			p = mesh.m_vertices[face[i]].Position();
			for (int j = 0; j < 3; j++)
				pp[i][j] = p[j];
		}
		UltraFace newFace = UltraFace(face[0], face[1], face[2]);
		mesh.m_faces.push_back(newFace);
	}
	PFDirectSolidQueryDestroy(query);
    PFEntityGetBoundsProperty(solid, PV_SOLID_PROP_BOUNDS, mesh.m_bounds);

}


int main(int argc, char* argv[])

{
	//std::wstring fileName = L"c:\\parts\\castor\\Spiral_20480.stl";
    //std::wstring fileName = L"c:\\parts\\industrial\\bracket.stl";
    //std::wstring fileName = L"C:\\Parts\\Castor\\Remeshed\\73986 LEVER_curve_sensitive.stl";
    std::wstring fileName = L"C:\\Parts\\Industrial\\Rocker Cover.stl";
    //std::wstring fileName = L"C:\\Parts\\Castor\\Coplanar\\coplanar_mesh1.stl";
    //std::wstring fileName = L"C:\\Parts\\Castor\\3dcross.stl";
    //std::wstring fileName = L"C:\\Parts\\Castor\\gauges.stl";
    //std::wstring fileName = L"C:\\Parts\\Castor\\less 6 another.stl";
    //std::wstring fileName = L"C:\\Parts\\Castor\\less6.stl";
    //std::wstring fileName = L"C:\\Parts\\Castor\\more6 another.stl";
    //std::wstring fileName = L"C:\\Parts\\Castor\\Part24.stl";
    //std::wstring fileName = L"C:\\Parts\\Castor\\FlachWithTwoHoles.stl";

    //std::wstring fileName = L"";

	PTInitialiseOpts initialise_options;
	PTEnvironment env = PV_ENTITY_NULL;
	PTEnvironmentOpts env_options;
	HWND window;
	PTDrawable drawable;
	PTWorld world;
	PTViewport vp;
	PTPoint vp_from = { 1000.0, 1000.0, 1000.0 };
	PTPoint vp_to = { 0.0, 0.0, 0.0 };
	PTVector vp_up = { 0.0, 0.0, 1.0 };

	double modelColor[3] = { 0.0, 0.0, 0.0 };
	double modelEdgeColor[3] = { 0.1, 0.1, 0.1 };
	PTNat32 modelTransparency = 80;
	bool modelViewEdges = true;
	bool modelView = true;
	double thinAreascolor[3] = { 1.0, 0.0, 0.0 };
	double thinAreasedgeColor[3] = { 0.1, 0.1, 0.1 };
	PTNat32 thinAreastransparency = 0;
	bool thinAreasViewEdges = false;
	bool thinAreasview = true;

    double thickAreascolor[3] = { 0.0, 0.5, 0.0 };
    double thickAreasedgeColor[3] = { 0.1, 0.1, 0.1 };
    PTNat32 thickAreastransparency = 0;
    bool thickAreasViewEdges = false;
    bool thickAreasview = true;
    
    double bgBottomColor[3] = { 0.2, 0.4, 0.6 };
	double bgTopColor[3] = { 0.1, 0.2, 0.3 };
	bool renderEdges = true;
	PTRenderStyle render_style;
	PTPolygonStyle poly_style;
	PTEdgeStyle edge_style;

	PTStatus status;


	PMInitInitialiseOpts(&initialise_options);
	status = PFInitialise(PV_LICENSE, &initialise_options);

#if JOURNAL_DEBUG
	PTStream journal_stream = PV_ENTITY_NULL;
	/* For debug builds journaling should be enabled for support  */
	/* Set journal file stream directly after initialising        */
	/* Polygonica                                                 */
	status = PFStreamFileOpen(PTPointer("c:\\temp\\PG_journal_ColorMeshSimplifier_2020_03_20_1.pgj"), PV_FILE_WRITE,
		NULL, &journal_stream);
	if (status == PV_STATUS_OK)
	{
		printf("Failed to start debug journal\n");
	}
	PFEntitySetEnumProperty(PV_ENTITY_NULL, PV_GLOBAL_PROP_JOURNAL_LEVEL, PV_JOURNAL_LEVEL_VERBOSE);
	PFEntitySetEntityProperty(PV_ENTITY_NULL, PV_GLOBAL_PROP_JOURNAL_STREAM, journal_stream);
#endif

	PMInitEnvironmentOpts(&env_options);
	status += PFEnvironmentCreate(&env_options, &env);

	window = PgWindowCreate(PTString("UltraMesh"), 600, 50, 1200, 800);

	status += PFDrawableCreate(env, window, NULL, &drawable);
	status += PFWorldCreate(env, NULL, &world);

	status += PFViewportCreate(world, &vp);

	PgWindowRegister(window, drawable, vp);
	PFEntitySetEnumProperty(vp, PV_VP_PROP_BG_MODE, PV_VP_BG_GRADUATED);
	PFEntitySetColourProperty(vp, PV_VP_PROP_BG_TOP_COL, PV_COLOUR_DOUBLE_RGB_ARRAY, bgTopColor);
	PFEntitySetColourProperty(vp, PV_VP_PROP_BG_BOTTOM_COL, PV_COLOUR_DOUBLE_RGB_ARRAY, bgBottomColor);

	status += PFDrawableRender(drawable, vp, PV_RENDER_MODE_BACKGROUND);

	printf("Reading file: %ls\n", fileName.c_str());

	PTSolid model = PV_ENTITY_NULL;
	PTSolid modifiedModel = PV_ENTITY_NULL;

	UltraMesh ultraMesh = UltraMesh();
    if (fileName.length() > 0)
        model = ReadSTL(env, fileName);
    else
    {
        //PFSolidCreateSphere(env, PTPoint{ 0.0, 0.0, 0.0 }, 50.0, 1, NULL, &model);
        //      PFSolidCreateCylinder(env, PTPoint{ 0.0, 0.0, 0.0 }, PTPoint{ 0.0, 0.0, 100.0 }, 20.0, 5, NULL, &model);
                PFSolidCreateFromBox(env, PTBounds{ -20.0, 20.0, -20.0, 20.0, -20.0, 20.0, }, NULL, &model);
        PTTransformMatrix mat;
        PMInitTransformMatrix(mat);
//        PFTransformMatrixScale(mat, 1, 0.1, 0.5);
        PFTransformMatrixRotate(mat, PTPoint{ 0.0, 0.0, 0.0 }, PTVector{ 0.0, 0.0, 1.0 }, 45);
        PFSolidTransform(model, mat);
    }
    if (REMESH)
    {
        PTSolidRemeshOpts initSolidRemeshOpts;
        PMInitSolidRemeshOpts(&initSolidRemeshOpts);
        initSolidRemeshOpts.keep_sharp_features = TRUE;
        initSolidRemeshOpts.remesh_limits = PV_REMESH_LIMIT_ERROR;
        initSolidRemeshOpts.keep_boundaries = TRUE;
        initSolidRemeshOpts.curvature_sensitive_remeshing = TRUE;
        initSolidRemeshOpts.error = 0.1;
        status += PFSolidRemesh(model, &initSolidRemeshOpts);
        WriteSTL(model, L"c:\\temp\\!meshed.stl");
    }
    auto start = std::chrono::high_resolution_clock::now();

	printf("Building mesh..\n");
	PTSolid2UltraMesh(model, ultraMesh);
    Bounds* bounds = ultraMesh.CalcBounds() ;
    VoxelVolume voxels = VoxelVolume(*bounds, VOXEL_SIZE);
    voxels.CalcBorder(ultraMesh.m_faces, ultraMesh.m_vertices);
    //std::set<std::array<int, 3>> border = voxels.Border();
    std::vector< Eigen::Vector3i> ijks;
    voxels.CalcSecond();
    bool layerCompleted = true;
    int layerIdx = 2;
    int maxLayer = round((double)MINIMAL_WALL_THICKNESS / VOXEL_SIZE) + 1;
    while (layerCompleted)
    {
        layerCompleted = ((layerIdx > maxLayer) || voxels.CalcLayer(layerIdx, layerIdx + 1));
        layerIdx++;
    }
    layerIdx--;
    voxels.CalcDepth();
    //voxels.Render(1, ijks);
    voxels.RenderByDepth(0.0, MINIMAL_WALL_THICKNESS / 2.0, ijks);
    //std::set<std::array<int, 3>> outside = voxels.Outside();
    PTSolid thinAreasvoxes = PV_ENTITY_NULL;
    for (auto& it : ijks)
    {
        PTBounds bounds = { it[0], it[0] + 1, it[1], it[1] + 1, it[2], it[2] + 1 };
        PTSolid brick;
        PFSolidCreateFromBox(env, bounds, NULL, &brick);
        if (thinAreasvoxes)
            PFSolidConcatenate(thinAreasvoxes, brick, TRUE, NULL);
        else
            PFSolidCopy(brick, NULL, &thinAreasvoxes);
    }
    PTSolid thickAreasvoxes = PV_ENTITY_NULL;
    //voxels.Render(layerIdx, ijks);
    voxels.RenderByDepth(MINIMAL_WALL_THICKNESS / 2.0, 999.0, ijks);
    //voxels.RenderByDepth(998, 999.0, ijks);
    for (auto& it : ijks)
    {
        PTBounds bounds = { it[0], it[0] + 1, it[1], it[1] + 1, it[2], it[2] + 1 };
        PTSolid brick;
        PFSolidCreateFromBox(env, bounds, NULL, &brick);
        if (thickAreasvoxes)
            PFSolidConcatenate(thickAreasvoxes, brick, TRUE, NULL);
        else
            PFSolidCopy(brick, NULL, &thickAreasvoxes);
    }
    PTTransformMatrix mat;
    PMInitTransformMatrix(mat);
    PTBounds bbx;
    memcpy(&bbx, bounds, sizeof(Bounds));
    PFTransformMatrixScale(mat, VOXEL_SIZE, VOXEL_SIZE, VOXEL_SIZE);
    PFTransformMatrixTranslate(mat, PTVector{ bbx[0] - VOXEL_SIZE / 2, bbx[2] - VOXEL_SIZE / 2, bbx[4] - VOXEL_SIZE / 2 });
    PFSolidTransform(thinAreasvoxes, mat);
    PFSolidTransform(thickAreasvoxes, mat);

	//ultraMesh.CalcFaces();
	//ultraMesh.MapEdges();
	//ultraMesh.CalcNormals(false);
 //   ultraMesh.CalcCurvature();
	//printf("Calculating buckets..\n");
	//ultraMesh.CalcBuckets();
	//int rays = 1;
	//printf("Tracing %d rays...  ", rays);
	//for (int i = 0; i < rays; i++)
	//{
	//	Eigen::Vector3d origin = { (rand() / RAND_MAX - 0.5) * 100, 
	//		(rand() / RAND_MAX - 0.5) * 100, 
	//		(rand() / RAND_MAX - 0.5) * 100 };
	//	Eigen::Vector3d direction = { (rand() / RAND_MAX - 0.5) * 2, 
	//		 (rand() / RAND_MAX - 0.5) * 2,
	//		 (rand() / RAND_MAX - 0.5) * 2 };
	//	ultraMesh.IntersectWithRay(origin, direction);
	//}
 //   UltraMesh modelMesh = ultraMesh;
 //   
 //   double halfThickness = MINIMAL_WALL_THICKNESS / 2.0;

 //   ultraMesh.OffsetBySkeleton(-halfThickness);
 //   //ultraMesh.Smooth();
 //   modelMesh.CalcThickness(ultraMesh);
    auto end = std::chrono::high_resolution_clock::now();
    printf("Completed in %3.3f seconds. \n", TIME_INTERVAL(end, start));
 //   modelMesh.CalcColors(MINIMAL_WALL_THICKNESS / 2, MINIMAL_WALL_THICKNESS  / 2);
 //   modelMesh.SaveAsVRML(L"c:/temp/!thickness.wrl");
 //   ultraMesh.SaveAsVRML(L"c:/temp/!skeleton.wrl");

	//modifiedModel = UltraMesh2PTSolid(ultraMesh, env);


	PTBounds modelBounds;

	PFEntityGetBoundsProperty(model, PV_SOLID_PROP_BOUNDS, modelBounds);


	PTWorldEntity modelEntity, wthinAreas = PV_ENTITY_NULL, wthickAreas = PV_ENTITY_NULL;

	//status += PFWorldAddEntity(world, model, &modelEntity);
    if (thinAreasvoxes)
        PFWorldAddEntity(world, thinAreasvoxes, &wthinAreas);
    if (thickAreasvoxes)
        PFWorldAddEntity(world, thickAreasvoxes, &wthickAreas);

	vp_to[0] = (modelBounds[0] + modelBounds[1]) / 2.0;
	vp_to[1] = (modelBounds[2] + modelBounds[3]) / 2.0;
	vp_to[2] = (modelBounds[4] + modelBounds[5]) / 2.0;
	vp_from[0] = vp_to[0];
	vp_from[1] = vp_to[1];
	vp_from[2] = vp_to[2] + modelBounds[5] - modelBounds[4];

	status += PFViewportSetPinhole(vp, &vp_from[0], &vp_to[0], &vp_up[0], PV_PROJ_ORTHOGRAPHIC, 500.0);
	status += PFViewportFit(vp, modelBounds);


	status += PFRenderStyleCreate(env, &render_style);
	poly_style = PFEntityGetEntityProperty(render_style, PV_RSTYLE_PROP_POLYGON_STYLE);
	edge_style = PFEntityGetEntityProperty(render_style, PV_RSTYLE_PROP_EDGE_STYLE);
	PFEntitySetColourProperty(poly_style, PV_PSTYLE_PROP_COLOUR, PV_COLOUR_DOUBLE_RGB_ARRAY, modelColor);
	PFEntitySetNat32Property(poly_style, PV_PSTYLE_PROP_TRANSPARENCY, modelTransparency);
	PFEntitySetColourProperty(edge_style, PV_ESTYLE_PROP_COLOUR, PV_COLOUR_DOUBLE_RGB_ARRAY, modelEdgeColor);
	PFEntitySetBooleanProperty(render_style, PV_RSTYLE_PROP_RENDER_EDGES, modelViewEdges);
	PFEntitySetDoubleProperty(render_style, PV_RSTYLE_PROP_EDGE_ANGLE, 0.0);
	PFEntitySetEntityProperty(modelEntity, PV_WENTITY_PROP_STYLE, render_style);
	PFEntitySetBooleanProperty(modelEntity, PV_WENTITY_PROP_VISIBLE, modelView);
	status += PFRenderStyleDestroy(render_style);

    if (wthinAreas)
    {
        status += PFRenderStyleCreate(env, &render_style);
        poly_style = PFEntityGetEntityProperty(render_style, PV_RSTYLE_PROP_POLYGON_STYLE);
        edge_style = PFEntityGetEntityProperty(render_style, PV_RSTYLE_PROP_EDGE_STYLE);
        PFEntitySetColourProperty(poly_style, PV_PSTYLE_PROP_COLOUR, PV_COLOUR_DOUBLE_RGB_ARRAY, thinAreascolor);
        PFEntitySetNat32Property(poly_style, PV_PSTYLE_PROP_TRANSPARENCY, thinAreastransparency);
        PFEntitySetColourProperty(edge_style, PV_ESTYLE_PROP_COLOUR, PV_COLOUR_DOUBLE_RGB_ARRAY, thinAreasedgeColor);
        PFEntitySetBooleanProperty(render_style, PV_RSTYLE_PROP_RENDER_EDGES, thinAreasViewEdges);
        PFEntitySetDoubleProperty(render_style, PV_RSTYLE_PROP_EDGE_ANGLE, 0.0);
        PFEntitySetEntityProperty(wthinAreas, PV_WENTITY_PROP_STYLE, render_style);
        PFEntitySetBooleanProperty(wthinAreas, PV_WENTITY_PROP_VISIBLE, thinAreasview);
        status += PFRenderStyleDestroy(render_style);
    }

    if (wthickAreas)
    {
        status += PFRenderStyleCreate(env, &render_style);
        poly_style = PFEntityGetEntityProperty(render_style, PV_RSTYLE_PROP_POLYGON_STYLE);
        edge_style = PFEntityGetEntityProperty(render_style, PV_RSTYLE_PROP_EDGE_STYLE);
        PFEntitySetColourProperty(poly_style, PV_PSTYLE_PROP_COLOUR, PV_COLOUR_DOUBLE_RGB_ARRAY, thickAreascolor);
        PFEntitySetNat32Property(poly_style, PV_PSTYLE_PROP_TRANSPARENCY, thickAreastransparency);
        PFEntitySetColourProperty(edge_style, PV_ESTYLE_PROP_COLOUR, PV_COLOUR_DOUBLE_RGB_ARRAY, thickAreasedgeColor);
        PFEntitySetBooleanProperty(render_style, PV_RSTYLE_PROP_RENDER_EDGES, thickAreasViewEdges);
        PFEntitySetDoubleProperty(render_style, PV_RSTYLE_PROP_EDGE_ANGLE, 0.0);
        PFEntitySetEntityProperty(wthickAreas, PV_WENTITY_PROP_STYLE, render_style);
        PFEntitySetBooleanProperty(wthickAreas, PV_WENTITY_PROP_VISIBLE, thickAreasview);
        status += PFRenderStyleDestroy(render_style);
    }




	/* Adjust viewport to fit solid and render */
	status += PFDrawableRender(drawable, vp, PV_RENDER_MODE_SOLID);

	PROMPT((char*)"Press any key to terminate.\n");

	PFTerminate();

	return 0;
}