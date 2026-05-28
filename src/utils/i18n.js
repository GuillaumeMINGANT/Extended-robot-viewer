/**
 * 国际化工具 - 支持中文和英文
 */

// 获取版本号（构建时会替换为实际版本号）
const APP_VERSION = typeof __APP_VERSION__ !== 'undefined' ? __APP_VERSION__ : '1.0.0';

export const translations = {
    'zh-CN': {
        // 顶部控制栏
        'visual': '视觉',
        'transparency': '透明',
        'collision': '碰撞',
        'com': '质心',
        'inertia': '惯量',
        'axes': '坐标轴',
        'jointAxes': '关节轴',
        'shadow': '阴影',
        'lighting': '光照',
        'upSelect': '上方向',
        'viewSelect': '视图',
        'viewFront': '前',
        'viewBack': '后',
        'viewLeft': '左',
        'viewRight': '右',
        'viewTop': '顶',
        'viewBottom': '底',
        'viewIso': '等轴测',
        'files': '文件',
        'joints': '关节',
        'controller': '控制器',
        'structure': '结构',
        'editor': '编辑器',
        'measure': '测量',
        'help': '帮助',
        'theme': '主题',
        'language': '语言',

        // 面板标题
        'fileList': '文件',
        'jointControl': '关节',
        'modelStructure': '结构',
        'codeEditor': '编辑',

        // 关节控制
        'radian': '弧度',
        'degree': '角度',
        'reset': '重置',
        'limits': '限位',

        // MuJoCo 仿真
        'mujocoReset': '重置',
        'mujocoSimulate': '仿真',
        'mujocoPause': '暂停',

        // 代码编辑器
        'reload': '重新加载',
        'download': '下载',
        'saved': '已保存',
        'unsaved': '未保存',
        'noFileOpen': '未打开文件',

        // 帮助对话框
        'helpTitle': `Robot Viewer v${APP_VERSION}`,
        'about': '关于',
        'aboutContent': 'Robot Viewer 是一个基于 Three.js 的网页端机器人模型 3D 查看器，提供直观的可视化界面，帮助您在浏览器中查看和分析机器人的结构、关节和物理属性，无需安装任何软件。<br><br>格式支持：URDF、Xacro、MJCF、USD（部分支持）<br>机器人类型：串联机器人结构（暂不支持并联机器人）<br><br>由 <strong>范子琦</strong> 开发。',
        'projectHome': '项目主页',
        'email': '邮箱',
        'myGithub': '我的GitHub',
        'operations': '操作指南',
        'leftDrag': '左键拖动',
        'rotateView': '旋转视角',
        'rightDrag': '右键拖动',
        'panView': '平移视角',
        'scroll': '滚轮',
        'zoom': '缩放视图',
        'clickModel': '点击模型',
        'controlJoint': '控制关节（可拖动）',
        'dragFile': '拖拽文件',
        'loadModel': '加载机器人模型',
        'contact': '联系方式',
        'support': '支持',

        // 其他
        'noFolder': '未加载文件夹',
        'noModel': '未加载模型',
        'load': '加载',
        'loadFiles': '加载文件',
        'loadFolder': '加载文件夹',
        'loadFromCatalog': '从目录加载',
        'catalogTitle': '机器人目录',
        'catalogSearch': '搜索机器人…',
        'catalogClose': '关闭',
        'catalogToggleOpen': '打开机器人目录',
        'catalogToggleClose': '关闭机器人目录',
        'catalogLoading': '正在加载目录…',
        'catalogLoadError': '无法加载目录',
        'catalogModelLoading': '正在加载模型…',
        'catalogModelError': '模型加载失败',
        'catalogNoResults': '没有匹配的机器人',
        'catalogBack': '品牌',
        'catalogCategoryAll': '全部',
        'catalogCategoryArm': '机械臂',
        'catalogCategoryBiped': '双足',
        'catalogCategoryDrone': '无人机',
        'catalogCategoryDualArm': '双臂',
        'catalogCategoryHand': '灵巧手',
        'catalogCategoryHumanoid': '人形',
        'catalogCategoryMobile': '移动',
        'catalogCategoryQuadruped': '四足',
        'catalogCategoryWheeled': '轮式',
        'catalogModelCountOne': '1 个型号',
        'catalogModelCountMany': '{n} 个型号',
        'catalogAttribution': '模型文件来自公共 CDN，可在构建设置中更换数据源。',

        'settingsTitle': '设置',
        'settingsClose': '关闭',
        'settingsOpenAria': '打开设置',
        'settingsToggleOpen': '打开设置',
        'settingsToggleClose': '关闭设置',
        'settingsAppearance': '外观',
        'settingsView': '视图',
        'settingsAngleUnit': '角度单位',
        'settingsThemeDark': '深色',
        'settingsThemeLight': '浅色',
        'settingsLangEn': 'English',
        'settingsLangZh': '中文',
        'settingsFloorGrid': '地面网格',
        'settingsPerformance': '性能',
        'orClickButton': '或点击下面的按钮加载',
        'noControllableJoints': '未找到可控制关节',
        'clickToEditMin': '点击编辑下限',
        'clickToEditMax': '点击编辑上限',
        'dropHint': '拖拽机器人模型文件或文件夹到页面任意位置',
        'dropHintSub': '支持 URDF, Xacro, MJCF 格式<br>支持拖拽文件夹以加载mesh文件',
        // Measure panel
        'measurePanel': '测量',
        'measureOverview': '概览',
        'measureLinks': 'Links',
        'measureLimbs': '关节识别',
        'measureJointIdentification': '关节识别',
        'measureDistance': '距离',
        'measureTotalMass': '总质量',
        'measureBoundingBox': '包围盒尺寸',
        'measureCenterOfMass': '重心位置',
        'measureStructure': '结构统计',
        'measureLinkName': 'Link名称',
        'measureParentJoint': '父关节',
        'measureChildJoints': '子关节',
        'measureLimbName': '肢体链',
        'measureLimbLinks': 'Links数',
        'measureLimbJoints': '关节数',
        'measureLimbLength': '长度',
        'measureLinkLength': 'Link长度',
        'measureLimitsDeg': '限位 (Deg)',
        'measureLimitsRad': '限位 (rad)',
        'measureLimitsMm': '限位 (mm)',
        'measureLimitsM': '限位 (m)',
        'measureTorque': '力矩 (Nm)',
        'measureVelocityRadS': '速度 (rad/s)',
        'measureVelocityRpm': '速度 (rpm)',
        'measureUnits': '单位',
        'measureUnitAngle': '角度',
        'measureUnitLinear': '长度',
        'measureUnitVelocity': '角速度',
        'measureUnitTorque': '力矩',
        'measureUnitSystemCustom': '自定义',
        'measurePresetMks': 'MKS（米、千克、秒）',
        'measurePresetCgs': 'CGS（厘米、克、秒）',
        'measurePresetMmgs': 'MMGS（毫米、克、秒）',
        'measurePresetIps': 'IPS（英寸、磅、秒）',
        'measureEditUnits': '编辑单位…',
        'measureLimitMin': '最小',
        'measureLimitMax': '最大',
        'measureNoLimbs': '未检测到关节',
        'measureNoJoints': '未检测到关节',
        'measureCopyTable': '复制表格',
        'measureDownloadTable': '下载 CSV',
        'measureTableCopied': '已复制到剪贴板',
        'measureCopyFailed': '复制失败',
        'measureJoint': '关节',
        'measureSelectJoint': '选择关节',
        'measureCompute': '测量距离',
        'measureClear': '清除',
        'measureTotalDistance': '总距离',
        'measureLimbCategory': '类别',
        'measureJointAxis': '关节轴',
        'measureJointAxisYaw': 'Yaw / 旋转',
        'measureJointAxisPitch': 'Pitch / 外展',
        'measureJointAxisRoll': 'Roll / 屈伸',
        'measureShowVisual': '显示',
        'measureRefresh': '更新',

        'graphHint': '拖动: 移动 | 滚轮: 缩放 | 右键: 隐藏/显示 | Ctrl+左键: 测量',
        'copyright': '© 2025 范子琦 版权所有。',

        // 模型信息
        'type': '类型',
        'links': 'Links',
        'joints': '关节',
        'controllable': '可控',
        'rootLink': '根Link',

        // 悬浮信息
        'linkName': 'Link名称',
        'jointName': '关节',
        'mass': '质量',
        'mergedLinks': '合并的Links',

        // 文件类型
        'model': '模型',
        'mesh': '网格',
        'link': '链接',

        // 单位
        'kg': 'kg',
        'rad': 'rad',
        'deg': 'deg',
        'm': 'm',

        // 状态消息
        'loading': '正在加载',
        'unsupportedFormat': '不支持的文件格式',
        'loadFailed': '加载失败',
        'noSupportedFiles': '未找到支持的文件（URDF, Xacro, MJCF, DAE, STL, OBJ）',
        'loadSuccess': '模型加载成功',
        'cannotLoadMesh': '无法加载 mesh 文件',

        // 编辑器消息
        'unsavedChanges': '您有未保存的更改，确定要关闭吗？',
        'newFile': '新文件.xml',
        'noFileToReload': '没有可重新加载的文件',
        'saveFirst': '请先保存为文件后再加载',
        'reloadingModel': '正在重新加载模型...',
        'modelReloaded': '模型已重新加载（未保存）',
        'reloadFailed': '重新加载失败',
        'downloadFailed': '下载失败',
        'fileDownloaded': '文件已下载',
        'emptyContent': '编辑器内容为空，无法加载',
        'fileType': '文件类型',

        // IK 控制器
        'ikEnable': '逆运动学',
        'ikHome': '初始姿态',
        'ikRandom': '随机姿态',
        'ikReachability': '可达性',
        'ikInfo': '求解器信息',
        'ikSolver': '求解器',
        'ikSolverDLSDesc': '阻尼最小二乘法 (Jacobian)',
        'ikSolverQPDesc': '二次规划 (任务空间)',
        'controllerDirect': '直接控制',
        'controllerIK': 'IK',
        'measureReachability': '可达性'
    },
    'en-US': {
        // Top control bar
        'visual': 'Visual',
        'transparency': 'Transparency',
        'collision': 'Collision',
        'com': 'COM',
        'inertia': 'Inertia',
        'axes': 'Axes',
        'jointAxes': 'Joint Axes',
        'shadow': 'Shadow',
        'lighting': 'Lighting',
        'upSelect': 'Up',
        'viewSelect': 'View',
        'viewFront': 'Front',
        'viewBack': 'Back',
        'viewLeft': 'Left',
        'viewRight': 'Right',
        'viewTop': 'Top',
        'viewBottom': 'Bottom',
        'viewIso': 'Isometric',
        'files': 'Files',
        'joints': 'Joints',
        'controller': 'Controller',
        'structure': 'Structure',
        'editor': 'Editor',
        'measure': 'Measure',
        'help': 'Help',
        'theme': 'Theme',
        'language': 'Language',

        // Panel titles
        'fileList': 'Files',
        'jointControl': 'Joints',
        'modelStructure': 'Structure',
        'codeEditor': 'Editor',

        // Joint control
        'radian': 'Radian',
        'degree': 'Degree',
        'reset': 'Reset',
        'limits': 'Limits',

        // MuJoCo simulation
        'mujocoReset': 'Reset',
        'mujocoSimulate': 'Simulate',
        'mujocoPause': 'Pause',

        // Code editor
        'reload': 'Reload',
        'download': 'Download',
        'saved': 'Saved',
        'unsaved': 'Unsaved',
        'noFileOpen': 'No File Open',

        // Help dialog
        'helpTitle': `Robot Viewer v${APP_VERSION}`,
        'about': 'About',
        'aboutContent': 'Robot Viewer is a web-based 3D viewer for robot models and scenes. Built on top of Three.js, it provides an intuitive interface for visualizing, editing, and simulating robots directly in the browser without any installation required. This tool helps you visualize and analyze robot structures, joints, and physical properties.<br><br>Format Support: URDF, Xacro, MJCF, USD (partial support)<br>Robot Types: Serial robot structures (parallel robots not currently supported)<br><br>Developed by <strong>Ziqi Fan</strong>.',
        'projectHome': 'Project Home',
        'email': 'Email',
        'myGithub': 'My GitHub',
        'operations': 'Operations',
        'leftDrag': 'Left Drag',
        'rotateView': 'Rotate View',
        'rightDrag': 'Right Drag',
        'panView': 'Pan View',
        'scroll': 'Scroll',
        'zoom': 'Zoom',
        'clickModel': 'Click Model',
        'controlJoint': 'Control Joint (Draggable)',
        'dragFile': 'Drag File',
        'loadModel': 'Load Robot Model',
        'contact': 'Contact',
        'support': 'Support',

        // Others
        'noFolder': 'No Folder Loaded',
        'noModel': 'No Model Loaded',
        'load': 'Load',
        'loadFiles': 'Load Files',
        'loadFolder': 'Load Folder',
        'loadFromCatalog': 'Load from catalog',
        'catalogTitle': 'Robot catalog',
        'catalogSearch': 'Search robots…',
        'catalogClose': 'Close',
        'catalogToggleOpen': 'Open robot catalog',
        'catalogToggleClose': 'Close robot catalog',
        'catalogLoading': 'Loading catalog…',
        'catalogLoadError': 'Could not load catalog',
        'catalogModelLoading': 'Loading model…',
        'catalogModelError': 'Failed to load model',
        'catalogNoResults': 'No robots match your search',
        'catalogBack': 'Brands',
        'catalogCategoryAll': 'All',
        'catalogCategoryArm': 'Arm',
        'catalogCategoryBiped': 'Biped',
        'catalogCategoryDrone': 'Drone',
        'catalogCategoryDualArm': 'Dual arm',
        'catalogCategoryHand': 'Hand',
        'catalogCategoryHumanoid': 'Humanoid',
        'catalogCategoryMobile': 'Mobile',
        'catalogCategoryQuadruped': 'Quadruped',
        'catalogCategoryWheeled': 'Wheeled',
        'catalogModelCountOne': '1 model',
        'catalogModelCountMany': '{n} models',
        'catalogAttribution': 'Model files are loaded from a public CDN. Override the source via build config.',

        'settingsTitle': 'Settings',
        'settingsClose': 'Close',
        'settingsOpenAria': 'Open settings',
        'settingsToggleOpen': 'Open settings',
        'settingsToggleClose': 'Close settings',
        'settingsAppearance': 'Appearance',
        'settingsView': 'View',
        'settingsAngleUnit': 'Angle unit',
        'settingsThemeDark': 'Dark',
        'settingsThemeLight': 'Light',
        'settingsLangEn': 'English',
        'settingsLangZh': '中文',
        'settingsFloorGrid': 'Floor grid',
        'settingsPerformance': 'Performance',
        'orClickButton': 'or click the button below to load',
        'noControllableJoints': 'No Controllable Joints Found',
        'clickToEditMin': 'Click to edit minimum',
        'clickToEditMax': 'Click to edit maximum',
        'dropHint': 'Drag and drop robot model files or folders anywhere',
        'dropHintSub': 'Supports URDF, Xacro, MJCF formats<br>Supports folder dragging to load mesh files',
        // Measure panel
        'measurePanel': 'Measure',
        'measureOverview': 'Overview',
        'measureLinks': 'Links',
        'measureLimbs': 'Joint Identification',
        'measureJointIdentification': 'Joint Identification',
        'measureDistance': 'Distance',
        'measureTotalMass': 'Total Mass',
        'measureBoundingBox': 'Bounding Box',
        'measureCenterOfMass': 'Center of Mass',
        'measureStructure': 'Structure',
        'measureLinkName': 'Link Name',
        'measureParentJoint': 'Parent Joint',
        'measureChildJoints': 'Child Joints',
        'measureLimbName': 'Limb Chain',
        'measureLimbLinks': 'Links',
        'measureLimbJoints': 'Joints',
        'measureLimbLength': 'Length',
        'measureLinkLength': 'Link length',
        'measureLimitsDeg': 'Limits (Deg)',
        'measureLimitsRad': 'Limits (rad)',
        'measureLimitsMm': 'Limits (mm)',
        'measureLimitsM': 'Limits (m)',
        'measureTorque': 'Torque (Nm)',
        'measureVelocityRadS': 'Velocity (rad/s)',
        'measureVelocityRpm': 'Velocity (rpm)',
        'measureUnits': 'Units',
        'measureUnitAngle': 'Angle',
        'measureUnitLinear': 'Length',
        'measureUnitVelocity': 'Velocity',
        'measureUnitTorque': 'Torque',
        'measureUnitSystemCustom': 'Custom',
        'measurePresetMks': 'MKS (meter, kilogram, second)',
        'measurePresetCgs': 'CGS (centimeter, gram, second)',
        'measurePresetMmgs': 'MMGS (millimeter, gram, second)',
        'measurePresetIps': 'IPS (inch, pound, second)',
        'measureEditUnits': 'Edit units…',
        'measureLimitMin': 'Min',
        'measureLimitMax': 'Max',
        'measureNoLimbs': 'No joints detected',
        'measureNoJoints': 'No joints detected',
        'measureCopyTable': 'Copy table',
        'measureDownloadTable': 'Download CSV',
        'measureTableCopied': 'Copied to clipboard',
        'measureCopyFailed': 'Copy failed',
        'measureJoint': 'Joint',
        'measureSelectJoint': 'Select joint',
        'measureCompute': 'Measure Distance',
        'measureClear': 'Clear',
        'measureTotalDistance': 'Total Distance',
        'measureLimbCategory': 'Category',
        'measureJointAxis': 'Joint Axis',
        'measureJointAxisYaw': 'Yaw / Rotation',
        'measureJointAxisPitch': 'Pitch / Abduction',
        'measureJointAxisRoll': 'Roll / Flexion',
        'measureShowVisual': 'Show',
        'measureRefresh': 'Update',

        'graphHint': 'Drag: Move | Scroll: Zoom | Right-click: Hide/Show | Ctrl+Click: Measure',
        'copyright': '© 2025 Ziqi Fan. All rights reserved.',

        // Model info
        'type': 'Type',
        'links': 'Links',
        'joints': 'Joints',
        'controllable': 'Controllable',
        'rootLink': 'Root Link',

        // Hover info
        'linkName': 'Link Name',
        'jointName': 'Joint',
        'mass': 'Mass',
        'mergedLinks': 'Merged Links',

        // File types
        'model': 'Model',
        'mesh': 'Mesh',
        'link': 'Link',

        // Units
        'kg': 'kg',
        'rad': 'rad',
        'deg': 'deg',
        'm': 'm',

        // Status messages
        'loading': 'Loading',
        'unsupportedFormat': 'Unsupported file format',
        'loadFailed': 'Load failed',
        'noSupportedFiles': 'No supported files found (URDF, Xacro, MJCF, DAE, STL, OBJ)',
        'loadSuccess': 'Model loaded successfully',
        'cannotLoadMesh': 'Cannot load mesh file',

        // Editor messages
        'unsavedChanges': 'You have unsaved changes. Are you sure you want to close?',
        'newFile': 'newfile.xml',
        'noFileToReload': 'No file to reload',
        'saveFirst': 'Please save the file first before loading',
        'reloadingModel': 'Reloading model...',
        'modelReloaded': 'Model reloaded (unsaved)',
        'reloadFailed': 'Reload failed',
        'downloadFailed': 'Download failed',
        'fileDownloaded': 'File downloaded',
        'emptyContent': 'Editor content is empty, cannot load',
        'fileType': 'File Type',

        // IK Controller
        'ikEnable': 'Inverse Kinematics',
        'ikHome': 'Home Pose',
        'ikRandom': 'Random Pose',
        'ikReachability': 'Reachability',
        'ikInfo': 'Solver Info',
        'ikSolver': 'Solver',
        'ikSolverDLSDesc': 'Damped Least-Squares (Jacobian)',
        'ikSolverQPDesc': 'Quadratic Programming (Task-space)',
        'controllerDirect': 'Direct',
        'controllerIK': 'IK',
        'measureReachability': 'Reachability'
    }
};

class I18n {
    constructor() {
        // 检测浏览器语言
        const browserLang = this.detectBrowserLanguage();
        // 从localStorage读取语言设置，如果没有则使用浏览器语言
        this.currentLang = localStorage.getItem('language') || browserLang;
    }

    /**
     * 检测浏览器语言
     */
    detectBrowserLanguage() {
        const lang = navigator.language || navigator.userLanguage;
        // 如果浏览器语言是中文（包括zh, zh-CN, zh-TW等），返回zh-CN
        if (lang.toLowerCase().startsWith('zh')) {
            return 'zh-CN';
        }
        // 否则默认返回英文
        return 'en-US';
    }

    /**
     * 获取翻译文本
     */
    t(key) {
        const lang = translations[this.currentLang] || translations['zh-CN'];
        return lang[key] || key;
    }

    /**
     * 切换语言
     */
    setLanguage(lang) {
        if (translations[lang]) {
            this.currentLang = lang;
            localStorage.setItem('language', lang);
            this.updatePageLanguage();
        }
    }

    /**
     * 获取当前语言
     */
    getCurrentLanguage() {
        return this.currentLang;
    }

    /**
     * 更新页面上所有带有data-i18n属性的元素
     */
    updatePageLanguage() {
        // 更新所有带有data-i18n属性的元素
        document.querySelectorAll('[data-i18n]').forEach(element => {
            const key = element.getAttribute('data-i18n');
            const text = this.t(key);

            // 如果是input或textarea，更新placeholder
            if (element.tagName === 'INPUT' || element.tagName === 'TEXTAREA') {
                element.placeholder = text;
            } else {
                // 如果包含HTML标签（如<br>），使用innerHTML
                if (text.includes('<br>') || text.includes('<strong>')) {
                    element.innerHTML = text;
                } else {
                    element.textContent = text;
                }
            }
        });

        document.querySelectorAll('[data-i18n-title]').forEach((element) => {
            const key = element.getAttribute('data-i18n-title');
            const text = this.t(key);
            element.title = text;
            if (element.tagName === 'BUTTON') {
                element.setAttribute('aria-label', text);
            }
        });

        // 更新HTML lang属性
        document.documentElement.lang = this.currentLang;
    }

    /**
     * 初始化页面语言
     */
    init() {
        this.updatePageLanguage();
    }
}

// 创建全局实例
export const i18n = new I18n();


