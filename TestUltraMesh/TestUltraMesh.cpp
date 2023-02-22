// TestUltraMesh.cpp : This file contains the 'main' function. Program execution begins and ends there.
//

#include "pch.h"
#include <iostream>
#include <chrono>
#include "../UltraMesh/Include/UltraMesh.h"
#include "../TestUltraMesh/window.h"
#include "pg/pgrender.h"
#include "C:/Eigen3.3.7/Eigen/Dense"


#define PROMPT(_str) PgWindowText(_str);
#define JOURNAL_DEBUG 0
#define VISUALIZE_STAGES 0
#define MINIMAL_WALL_THICKNESS 6
#define VOXEL_SIZE 0.5
#define ROTATION 0
#define REMESH false
#define TIME_INTERVAL(end, start) double(std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count()) / 1000.0

//#define FILE_NAME  L"c:\\parts\\castor\\Spiral_20480.stl"
#define FILE_NAME  L"c:\\parts\\industrial\\bracket.stl"
//#define FILE_NAME  L"C:\\Parts\\Castor\\Remeshed\\73986 LEVER_curve_sensitive.stl"
//#define FILE_NAME  L"C:\\Parts\\Industrial\\Rocker Cover.stl"
//#define FILE_NAME  L"C:\\Parts\\Castor\\Coplanar\\coplanar_mesh1.stl"
//#define FILE_NAME  L"C:\\Parts\\Castor\\3dcross.stl"
//#define FILE_NAME  L"C:\\Parts\\Castor\\gauges.stl"
//#define FILE_NAME  L"C:\\Parts\\Castor\\less 6 another.stl"
//#define FILE_NAME  L"C:\\Parts\\Castor\\less6.stl"
//#define FILE_NAME  L"C:\\Parts\\Castor\\more6 another.stl"
//#define FILE_NAME  L"C:\\Parts\\Castor\\Part24.stl"
//#define FILE_NAME  L"C:\\Parts\\Castor\\FlachWithTwoHoles.stl"
//#define FILE_NAME L""


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


	PTInitialiseOpts initialise_options;
	PTEnvironment env = PV_ENTITY_NULL;
	PTEnvironmentOpts env_options;
	HWND window;
	PTDrawable drawable;
	PTWorld world;
	PTViewport vp;
	PTPoint vp_from = { 0.0, 0.0, 1000.0 };
	PTPoint vp_to = { 0.0, 0.0, 0.0 };
	PTVector vp_up = { 0.0, 1.0, 0.0 };

	double modelColor[3] = { 0.5, 0.5, 0.5 };
	double modelEdgeColor[3] = { 0.5, 0.5, 0.5 };
	PTNat32 modelTransparency = 100;
	bool modelViewEdges = true;
	bool modelView = true;

	double thinAreascolor[3] = { 1.0, 0.0, 0.0 };
	double thinAreasedgeColor[3] = { 0.1, 0.0, 0.0 };
	PTNat32 thinAreastransparency = 0;
	bool thinAreasViewEdges = false;
	bool thinAreasview = true;

    double thickAreascolor[3] = { 0.0, 0.8, 0.0 };
    double thickAreasedgeColor[3] = { 0.0, 0.1, 0.0 };
    PTNat32 thickAreastransparency = 0;
    bool thickAreasViewEdges = false;
    bool thickAreasview = true;

    double transientAreascolor[3] = { 1.0, 1.0, 0.0 };
    double transientAreasedgeColor[3] = { 0.1, 0.06, 0.0 };
    PTNat32 transientAreastransparency = 0;
    bool transientAreasViewEdges = false;
    bool transientAreasview = true;

    
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
    std::wstring fileName = FILE_NAME;

	printf("Reading file: %ls\n", fileName.c_str());

	PTSolid model = PV_ENTITY_NULL;
	PTSolid modifiedModel = PV_ENTITY_NULL;

	UltraMesh ultraMesh = UltraMesh();
    if (fileName.length() > 0)
        model = ReadSTL(env, fileName);
    else
    {
        PFSolidCreateSphere(env, PTPoint{ 0.0, 0.0, 0.0 }, 50.0, 5, NULL, &model);
        //      PFSolidCreateCylinder(env, PTPoint{ 0.0, 0.0, 0.0 }, PTPoint{ 0.0, 0.0, 100.0 }, 20.0, 5, NULL, &model);
        //        PFSolidCreateFromBox(env, PTBounds{ -20.0, 20.0, -20.0, 20.0, -20.0, 20.0, }, NULL, &model);
     }
    PTTransformMatrix mat;
    PMInitTransformMatrix(mat);
    //        PFTransformMatrixScale(mat, 1, 0.1, 0.5);
    PFTransformMatrixRotate(mat, PTPoint{ 0.0, 0.0, 0.0 }, PTVector{ 0.0, 0.0, 1.0 }, ROTATION);
    PFSolidTransform(model, mat);
    if (REMESH)
    {
        PTSolidRemeshOpts initSolidRemeshOpts;
        PMInitSolidRemeshOpts(&initSolidRemeshOpts);
        initSolidRemeshOpts.curvature_sensitive_remeshing = TRUE;
        initSolidRemeshOpts.remesh_limits = PV_REMESH_LIMIT_ERROR;
        initSolidRemeshOpts.multithread = TRUE;
        initSolidRemeshOpts.error = 0.1;
        status += PFSolidRemesh(model, &initSolidRemeshOpts);
        WriteSTL(model, L"c:\\temp\\!meshed.stl");
    }
    auto start = std::chrono::high_resolution_clock::now();

	printf("Building mesh..\n");
	PTSolid2UltraMesh(model, ultraMesh);
    Bounds* bounds = ultraMesh.CalcBounds() ;

    printf("Voxelizing...\n");
    VoxelVolume voxels = VoxelVolume(*bounds, VOXEL_SIZE);
    voxels.CalcSurfaceLayer(ultraMesh.m_faces, ultraMesh.m_vertices);
    std::set<std::array<int, 3>> border = voxels.Border();
    printf("\nSurface Layer... voxels: %d\n", border.size());
    //voxels.CalcOutside();

    int n = 0;
    printf("Calc Layer #2...");
    n = voxels.CalcSecond(false);
    printf(" voxels: %d\n", n);
    // loop over layers 2..maxLayer
    bool layerCompleted = false;
    int layerIdx = 2;
    // define maximal voxel layer by desired wall thickness
    int maxLayer = round((double)MINIMAL_WALL_THICKNESS / VOXEL_SIZE) + 1;
    while (!layerCompleted)
    {
        printf("Calc Layer #%d...", layerIdx);
        n = voxels.CalcLayer(layerIdx, layerIdx + 1, false);
        printf(" voxels: %d\n", n);
        layerCompleted = ((layerIdx > maxLayer) || (n < 1));
        layerIdx++;
    }
    layerIdx--;
 
    printf("Calc Depth Values...\n");
    voxels.CalcDepth(false);
    //voxels.Render(1, ijks);
    std::vector< Eigen::Vector3i> voxelsIJK;
    voxels.ClassifyByDepth(0.0, MINIMAL_WALL_THICKNESS / 2.0, VOXEL_FLAG_THIN);
    voxels.ClassifyByDepth(MINIMAL_WALL_THICKNESS / 2.0, 999.0, VOXEL_FLAG_THICK);
    for (int i = 0; i < maxLayer / 2; i++)
      voxels.AdjustNeighboringFlags();
    //std::set<std::array<int, 3>> outside = voxels.Outside();
    voxels.RenderByFlag(VOXEL_FLAG_THIN, voxelsIJK);
    PTSolid thinAreasvoxels = PV_ENTITY_NULL;
    for (auto& it : voxelsIJK)
    {
        PTBounds bounds = { it[0], it[0] + 1, it[1], it[1] + 1, it[2], it[2] + 1 };
        PTSolid brick;
        PFSolidCreateFromBox(env, bounds, NULL, &brick);
        if (thinAreasvoxels)
            PFSolidConcatenate(thinAreasvoxels, brick, TRUE, NULL);
        else
            PFSolidCopy(brick, NULL, &thinAreasvoxels);
    }

    voxels.RenderByFlag(VOXEL_FLAG_THICK, voxelsIJK);
    PTSolid thickAreasvoxels = PV_ENTITY_NULL;

    for (auto& it : voxelsIJK)
    {
        PTBounds bounds = { it[0], it[0] + 1, it[1], it[1] + 1, it[2], it[2] + 1 };
        PTSolid brick;
        PFSolidCreateFromBox(env, bounds, NULL, &brick);
        if (thickAreasvoxels)
            PFSolidConcatenate(thickAreasvoxels, brick, TRUE, NULL);
        else
            PFSolidCopy(brick, NULL, &thickAreasvoxels);
    }
 
    voxels.RenderByFlag(VOXEL_FLAG_TRANSIENT, voxelsIJK);
    PTSolid transientAreasvoxels = PV_ENTITY_NULL;

    for (auto& it : voxelsIJK)
    {
        PTBounds bounds = { it[0], it[0] + 1, it[1], it[1] + 1, it[2], it[2] + 1 };
        PTSolid brick;
        PFSolidCreateFromBox(env, bounds, NULL, &brick);
        if (transientAreasvoxels)
            PFSolidConcatenate(transientAreasvoxels, brick, TRUE, NULL);
        else
            PFSolidCopy(brick, NULL, &transientAreasvoxels);
    }
    PMInitTransformMatrix(mat);
    PTBounds bbx;
    memcpy(&bbx, bounds, sizeof(Bounds));
    PFTransformMatrixScale(mat, VOXEL_SIZE, VOXEL_SIZE, VOXEL_SIZE);
    PFTransformMatrixTranslate(mat, PTVector{ bbx[0] - VOXEL_SIZE * 1.5, bbx[2] - VOXEL_SIZE * 1.5, bbx[4] - VOXEL_SIZE * 1.5 });
    PFSolidTransform(thinAreasvoxels, mat);
    PFSolidTransform(thickAreasvoxels, mat);
    PFSolidTransform(transientAreasvoxels, mat);
   auto end = std::chrono::high_resolution_clock::now();
    printf("Completed in %3.3f seconds. \n", TIME_INTERVAL(end, start));


	PTBounds modelBounds;

	PFEntityGetBoundsProperty(model, PV_SOLID_PROP_BOUNDS, modelBounds);


	PTWorldEntity modelEntity = PV_ENTITY_NULL, wthinAreas = PV_ENTITY_NULL, wthickAreas = PV_ENTITY_NULL, wtransientAreas = PV_ENTITY_NULL;


	status += PFWorldAddEntity(world, model, &modelEntity);
 
	vp_to[0] = (modelBounds[0] + modelBounds[1]) / 2.0;
	vp_to[1] = (modelBounds[2] + modelBounds[3]) / 2.0;
	vp_to[2] = (modelBounds[4] + modelBounds[5]) / 2.0;
	vp_from[0] = vp_to[0];
	vp_from[1] = vp_to[1];
	vp_from[2] = vp_to[2] + modelBounds[5] - modelBounds[4];

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

    status += PFViewportSetPinhole(vp, &vp_from[0], &vp_to[0], &vp_up[0], PV_PROJ_ORTHOGRAPHIC, 500.0);
    status += PFViewportFit(vp, modelBounds);

    PROMPT((char*)"Press any key to continue.\n");
    if (VISUALIZE_STAGES)
    {
        layerIdx = 1;
        std::vector<PTWorldEntity> wlayers;
        while (layerIdx <= maxLayer)
        {
            voxelsIJK.clear();
            double factor = 0;// double(maxLayer - layerIdx) / maxLayer;
            voxels.Render(layerIdx, voxelsIJK);
            PTSolid layer = PV_ENTITY_NULL;
            for (auto& it : voxelsIJK)
            {
                PTBounds bounds = { it[0], it[0] + 1, it[1], it[1] + 1, it[2], it[2] + 1 };
                PTSolid brick;
                PFSolidCreateFromBox(env, bounds, NULL, &brick);
                if (layer)
                    PFSolidConcatenate(layer, brick, TRUE, NULL);
                else
                    PFSolidCopy(brick, NULL, &layer);
            }
            PTTransformMatrix mat;
            PMInitTransformMatrix(mat);
            PTBounds bbx;
            memcpy(&bbx, bounds, sizeof(Bounds));
            PFTransformMatrixScale(mat, VOXEL_SIZE, VOXEL_SIZE, VOXEL_SIZE);
            PFTransformMatrixTranslate(mat, PTVector{ bbx[0] - VOXEL_SIZE * 1.5, bbx[2] - VOXEL_SIZE * 1.5, bbx[4] - VOXEL_SIZE * 1.5 });
            PFSolidTransform(layer, mat);

            PTWorldEntity wLayer = PV_ENTITY_NULL;
            PFWorldAddEntity(world, layer, &wLayer);
            //wlayers.push_back(wLayer);
            status += PFRenderStyleCreate(env, &render_style);
            poly_style = PFEntityGetEntityProperty(render_style, PV_RSTYLE_PROP_POLYGON_STYLE);
            edge_style = PFEntityGetEntityProperty(render_style, PV_RSTYLE_PROP_EDGE_STYLE);
            double color[3] = { 1.0 - factor, 1.0 - factor, 1.0 - factor };
            PFEntitySetColourProperty(poly_style, PV_PSTYLE_PROP_COLOUR, PV_COLOUR_DOUBLE_RGB_ARRAY, color);
            PTNat32 transparency = 0;// (int)(100.0 * factor);
            printf("Transparency: %d\n", transparency);
            PFEntitySetNat32Property(poly_style, PV_PSTYLE_PROP_TRANSPARENCY, transparency);
            PFEntitySetColourProperty(edge_style, PV_ESTYLE_PROP_COLOUR, PV_COLOUR_DOUBLE_RGB_ARRAY, color);
            PFEntitySetBooleanProperty(render_style, PV_RSTYLE_PROP_RENDER_EDGES, TRUE);
            PFEntitySetDoubleProperty(render_style, PV_RSTYLE_PROP_EDGE_ANGLE, 90.0);
            PFEntitySetEntityProperty(wLayer, PV_WENTITY_PROP_STYLE, render_style);
            PFEntitySetBooleanProperty(wLayer, PV_WENTITY_PROP_VISIBLE, modelView);
            status += PFRenderStyleDestroy(render_style);
            char buf[128];
            sprintf_s(buf, "Layer %d of %d, press any key to continue ", layerIdx, maxLayer);
            PROMPT(buf);
            layerIdx++;
            PFWorldRemoveEntity(wLayer);
            //if (wlayers.size() > 2)
            //{
            //    PFWorldRemoveEntity(wlayers[0]);
            //    wlayers.erase(wlayers.begin());
            //}

        }

        for (PTWorldEntity wlayer : wlayers)
            PFWorldRemoveEntity(wlayer);
        wlayers.clear();

    }
    if (thinAreasvoxels)
        PFWorldAddEntity(world, thinAreasvoxels, &wthinAreas);
    if (thickAreasvoxels)
        PFWorldAddEntity(world, thickAreasvoxels, &wthickAreas);
    if (transientAreasvoxels)
        PFWorldAddEntity(world, transientAreasvoxels, &wtransientAreas);

    
 
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

    if (wtransientAreas)
    {
        status += PFRenderStyleCreate(env, &render_style);
        poly_style = PFEntityGetEntityProperty(render_style, PV_RSTYLE_PROP_POLYGON_STYLE);
        edge_style = PFEntityGetEntityProperty(render_style, PV_RSTYLE_PROP_EDGE_STYLE);
        PFEntitySetColourProperty(poly_style, PV_PSTYLE_PROP_COLOUR, PV_COLOUR_DOUBLE_RGB_ARRAY, transientAreascolor);
        PFEntitySetNat32Property(poly_style, PV_PSTYLE_PROP_TRANSPARENCY, transientAreastransparency);
        PFEntitySetColourProperty(edge_style, PV_ESTYLE_PROP_COLOUR, PV_COLOUR_DOUBLE_RGB_ARRAY, transientAreasedgeColor);
        PFEntitySetBooleanProperty(render_style, PV_RSTYLE_PROP_RENDER_EDGES, transientAreasViewEdges);
        PFEntitySetDoubleProperty(render_style, PV_RSTYLE_PROP_EDGE_ANGLE, 0.0);
        PFEntitySetEntityProperty(wtransientAreas, PV_WENTITY_PROP_STYLE, render_style);
        PFEntitySetBooleanProperty(wtransientAreas, PV_WENTITY_PROP_VISIBLE, transientAreasview);
        status += PFRenderStyleDestroy(render_style);
    }



	/* Adjust viewport to fit solid and render */
	status += PFDrawableRender(drawable, vp, PV_RENDER_MODE_SOLID);

	PROMPT((char*)"Press any key to continue.\n");
    PFWorldRemoveEntity(wthinAreas);
    PFWorldRemoveEntity(wthickAreas);

	PFTerminate();

	return 0;
}